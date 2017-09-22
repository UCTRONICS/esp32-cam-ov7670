/* Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <byteswap.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "soc/spi_reg.h"
#include "driver/hspi.h"
#include "soc/gpio_reg.h"
#include "esp_attr.h"

#include "soc/gpio_struct.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "camera.h"

#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/api.h"
#include "bitmap.h"

#define WIFI_PASSWORD CONFIG_WIFI_PASSWORD
#define WIFI_SSID     CONFIG_WIFI_SSID
#define CAMERA_PIXEL_FORMAT CAMERA_PF_RGB565
//#define CAMERA_PIXEL_FORMAT CAMERA_PF_YUV422
#define CAMERA_FRAME_SIZE CAMERA_FS_QQVGA

static const char* TAG = "ESP-CAM";
static EventGroupHandle_t espilicam_event_group;
EventBits_t uxBits;
const int MOVIEMODE_ON_BIT = BIT0;

bool is_moviemode_on()
{
    return (xEventGroupGetBits(espilicam_event_group) & MOVIEMODE_ON_BIT) ? 1 : 0;
}

static void set_moviemode(bool c) {
    if (is_moviemode_on() == c) {
        return;
    } else {
      if (c) {
      xEventGroupSetBits(espilicam_event_group, MOVIEMODE_ON_BIT);
      } else {
      xEventGroupClearBits(espilicam_event_group, MOVIEMODE_ON_BIT);
      }
    }
}

// CAMERA CONFIG
static camera_pixelformat_t s_pixel_format;
static camera_config_t config = {
    .ledc_channel = LEDC_CHANNEL_0,
    .ledc_timer = LEDC_TIMER_0,
    .pin_d0 = CONFIG_D0,
    .pin_d1 = CONFIG_D1,
    .pin_d2 = CONFIG_D2,
    .pin_d3 = CONFIG_D3,
    .pin_d4 = CONFIG_D4,
    .pin_d5 = CONFIG_D5,
    .pin_d6 = CONFIG_D6,
    .pin_d7 = CONFIG_D7,
    .pin_xclk = CONFIG_XCLK,
    .pin_pclk = CONFIG_PCLK,
    .pin_vsync = CONFIG_VSYNC,
    .pin_href = CONFIG_HREF,
    .pin_sscb_sda = CONFIG_SDA,
    .pin_sscb_scl = CONFIG_SCL,
    .pin_reset = CONFIG_RESET,
    .xclk_freq_hz = CONFIG_XCLK_FREQ,
   // .test_pattern_enabled = CONFIG_ENABLE_TEST_PATTERN,
    };

static camera_model_t camera_model;


// DISPLAY LOGIC
static inline uint8_t clamp(int n)
{
    n = n>255 ? 255 : n;
    return n<0 ? 0 : n;
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
static inline uint16_t ILI9341_color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

uint16_t get_grayscale_pixel_as_565(uint8_t pix) {
    // R = (img[n]&248)<<8; // 5 bit cao cua Y
    // G = (img[n]&252)<<3; // 6 bit cao cua Y
    // B = (img[n]&248)>>3; // 5 bit cao cua Y
    uint16_t graypixel=((pix&248)<<8)|((pix&252)<<3)|((pix&248)>>3);
    return graypixel;

}

// integers instead of floating point...
static inline uint16_t fast_yuv_to_rgb565(int y, int u, int v) {
int a0 = 1192 * (y - 16);
int a1 = 1634 * (v - 128);
int a2 = 832 * (v - 128);
int a3 = 400 * (u - 128);
int a4 = 2066 * (u - 128);
int r = (a0 + a1) >> 10;
int g = (a0 - a2 - a3) >> 10;
int b = (a0 + a4) >> 10;
return ILI9341_color565(clamp(r),clamp(g),clamp(b));

}

// fast but uses floating points...
static inline uint16_t fast_pascal_to_565(int Y, int U, int V) {
  uint8_t r, g, b;
  r = clamp(1.164*(Y-16) + 1.596*(V-128));
  g = clamp(1.164*(Y-16) - 0.392*(U-128) - 0.813*(V-128));
  b = clamp(1.164*(Y-16) + 2.017*(U-128));
  return ILI9341_color565(r,g,b);
}

//Warning: This gets squeezed into IRAM.
volatile static uint32_t *currFbPtr __attribute__ ((aligned(4))) = NULL;

inline uint8_t unpack(int byteNumber, uint32_t value) {
    return (value >> (byteNumber * 8));
}

// camera code

const static char http_hdr[] = "HTTP/1.1 200 OK\r\n";
const static char http_stream_hdr[] =
        "Content-type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n\r\n";
const static char http_jpg_hdr[] =
        "Content-type: image/jpg\r\n\r\n";
const static char http_pgm_hdr[] =
        "Content-type: image/x-portable-graymap\r\n\r\n";
const static char http_stream_boundary[] = "--123456789000000000000987654321\r\n";
const static char http_bitmap_hdr[] =
        "Content-type: image/bitmap\r\n\r\n";
const static char http_yuv422_hdr[] =
        "Content-Disposition: attachment; Content-type: application/octet-stream\r\n\r\n";

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;
static ip4_addr_t s_ip_addr;


// command parser...
#include "smallargs.h"

#define UNUSED(x) ((void)x)
#define RESPONSE_BUFFER_LEN 256
#define CMD_BUFFER_LEN 128

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            s_ip_addr = event->event_info.got_ip.ip_info.ip;
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            /* This is a workaround as ESP32 WiFi libs don't currently
             auto-reassociate. */
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void init_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    //printf("CONFIG_WIFI_PASSWORD IS %s\n",WIFI_PASSWORD);
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    //ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_set_ps(WIFI_PS_NONE) );
    ESP_LOGI(TAG, "Connecting to \"%s\"", wifi_config.sta.ssid);
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected");
}


// COMMAND PARSER
uint8_t getHexVal(char c)
{
   if(c >= '0' && c <= '9')
     return (uint8_t)(c - '0');
   else
     return (uint8_t)(c-'A'+10);
}

static void convert_fb32bit_line_to_bmp565(uint32_t *srcline, uint8_t *destline, const camera_pixelformat_t format) {

  uint16_t pixel565 = 0;
  uint16_t pixel565_2 = 0;
  uint32_t long2px = 0;
  uint16_t *sptr;
  int current_src_pos=0, current_dest_pos=0;
  for (int current_pixel_pos = 0; current_pixel_pos < 160; current_pixel_pos += 2)
  {
    current_src_pos = current_pixel_pos/2;
    long2px = srcline[current_src_pos];
    if (format == CAMERA_PF_YUV422) {
        uint8_t y1, y2, u, v;
        y1 = unpack(0,long2px);
        v = unpack(1,long2px);;
        y2 = unpack(2,long2px);
        u = unpack(3,long2px);

        pixel565 = fast_yuv_to_rgb565(y1,u,v);
        pixel565_2 = fast_yuv_to_rgb565(y2,u,v);

        sptr = &destline[current_dest_pos];
        *sptr = pixel565;
        sptr = &destline[current_dest_pos+2];
        *sptr = pixel565_2;
        current_dest_pos += 4;

    } else if (format == CAMERA_PF_RGB565) {
      pixel565 =  (unpack(2,long2px) << 8) | unpack(3,long2px);
      pixel565_2 = (unpack(0,long2px) << 8) | unpack(1,long2px);

      sptr = &destline[current_dest_pos];
      *sptr = pixel565;
      sptr = &destline[current_dest_pos+2];
      *sptr = pixel565_2;
      current_dest_pos += 4;
    }
  }
}


// TODO: handle http request while videomode on

static void http_server_netconn_serve(struct netconn *conn)
{
    struct netbuf *inbuf;
    char *buf;
    u16_t buflen;
    err_t err;
    //Read the data from the port, blocking if nothing yet there.We assume the request (the part we care about) is in one netbuf
    err = netconn_recv(conn, &inbuf);
    if (err == ERR_OK) {
        netbuf_data(inbuf, (void**) &buf, &buflen);
        /* Is this an HTTP GET command? (only check the first 5 chars, since
         there are other formats for GET, and we're keeping it very simple )*/
        if (buflen >= 5 && buf[0] == 'G' && buf[1] == 'E' && buf[2] == 'T' && buf[3] == ' ' && buf[4] == '/') {
            printf("000\n");
            // disable videomode (autocapture) to allow streaming...
            bool s_moviemode = is_moviemode_on();
            set_moviemode(false);
            /* Send the HTTP header
             * subtract 1 from the size, since we dont send the \0 in the string
             * NETCONN_NOCOPY: our data is const static, so no need to copy it
             */
            netconn_write(conn, http_hdr, sizeof(http_hdr) - 1,NETCONN_NOCOPY);
            //check if a stream is requested.
            if (buf[5] == 's') {
                printf("00\n");
                //Send mjpeg stream header
                err = netconn_write(conn, http_stream_hdr, sizeof(http_stream_hdr) - 1,NETCONN_NOCOPY);
                ESP_LOGD(TAG, "Stream started.");
                ESP_LOGD(TAG, "write http_stream hdr  = %d", err);
                //Run while everyhting is ok and connection open.
                while(err == ERR_OK) {
                    printf("01\n");
                    ESP_LOGD(TAG, "Capture frame");
                    //PAUSE_DISPLAY = true;
                    //err = camera_run();
                    //PAUSE_DISPLAY = false;
                    if (err != ESP_OK) {
                        ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
                    } else {
                        ESP_LOGD(TAG, "Done");
                        //stream an image..
                        if((s_pixel_format == CAMERA_PF_RGB565) || (s_pixel_format == CAMERA_PF_YUV422)) {
                            printf("02\n");
                            // write mime boundary start
                            err = netconn_write(conn, http_bitmap_hdr, sizeof(http_bitmap_hdr) - 1,NETCONN_NOCOPY);
                            // write bitmap header
                            char *bmp = bmp_create_header565(camera_get_fb_width(), camera_get_fb_height());
                            err = netconn_write(conn, bmp, sizeof(bitmap565), NETCONN_NOCOPY);
                            free(bmp);
                            // convert framebuffer on the fly...
                            // only rgb and yuv...
                            uint8_t s_line[160*2];
                            uint32_t *fbl;
                            for (int i = 0; i < 120; i++) {
                                fbl = &currFbPtr[(i*160)/2];  //(i*(320*2)/4); // 4 bytes for each 2 pixel / 2 byte read..
                                convert_fb32bit_line_to_bmp565(fbl, s_line,s_pixel_format);
                                err = netconn_write(conn, s_line, 160*2,NETCONN_COPY);
                            }
                        }else {
                            printf("03\n");
                            // stream jpeg
                            err = netconn_write(conn, http_jpg_hdr, sizeof(http_jpg_hdr) - 1,NETCONN_NOCOPY);
                            if(err == ERR_OK)
                                err = netconn_write(conn, camera_get_fb(), camera_get_data_size(),NETCONN_COPY);
                        }
                        if(err == ERR_OK){
                            //Send boundary to next jpeg
                            printf("04\n");
                            err = netconn_write(conn, http_stream_boundary,sizeof(http_stream_boundary) -1, NETCONN_NOCOPY);
                        }
                        vTaskDelay(30 / portTICK_RATE_MS);
                    }
                }
                ESP_LOGD(TAG, "Stream ended.");
                ESP_LOGI(TAG, "task stack: %d", uxTaskGetStackHighWaterMark(NULL));
            } else {

                //CAMER_PF_JPEG
                if (s_pixel_format == CAMERA_PF_JPEG) {
                printf("0\n");
                    netconn_write(conn, http_jpg_hdr, sizeof(http_jpg_hdr) - 1, NETCONN_NOCOPY);

                //CAMERA_PF_GRAYSCALE
                } else if (s_pixel_format == CAMERA_PF_GRAYSCALE) {
                printf("1\n");
                    netconn_write(conn, http_pgm_hdr, sizeof(http_pgm_hdr) - 1, NETCONN_NOCOPY);
                    if (memcmp(&buf[5], "pgm", 3) == 0) {
                        char pgm_header[32];
                        snprintf(pgm_header, sizeof(pgm_header), "P5 %d %d %d\n", camera_get_fb_width(), camera_get_fb_height(), 255);
                        netconn_write(conn, pgm_header, strlen(pgm_header), NETCONN_COPY);
                    }else {
                        char outstr[120];
                        get_image_mime_info_str(outstr);
                        netconn_write(conn, outstr, sizeof(outstr) - 1, NETCONN_NOCOPY);
                        //netconn_write(conn, http_yuv422_hdr, sizeof(http_yuv422_hdr) - 1, NETCONN_NOCOPY);
                    }

                //CAMERA_PF_RGB565
                } else if (s_pixel_format == CAMERA_PF_RGB565) {
                printf("2\n");
                    netconn_write(conn, http_bitmap_hdr, sizeof(http_bitmap_hdr) - 1, NETCONN_NOCOPY);
                    if (memcmp(&buf[5], "bmp", 3) == 0) {
                        char *bmp = bmp_create_header565(camera_get_fb_width(), camera_get_fb_height());
                        err = netconn_write(conn, bmp, sizeof(bitmap565), NETCONN_COPY);
                        free(bmp);
                    } else {
                        char outstr[120];
                        get_image_mime_info_str(outstr);
                        netconn_write(conn, outstr, sizeof(outstr) - 1, NETCONN_NOCOPY);
                        //netconn_write(conn, http_yuv422_hdr, sizeof(http_yuv422_hdr) - 1, NETCONN_NOCOPY);
                    }

                //CAMERA_PF_YUV422
                } else if (s_pixel_format == CAMERA_PF_YUV422) {
                printf("3\n");
                    if (memcmp(&buf[5], "bmp", 3) == 0) {
                        printf("a\n");
                        //PAUSE_DISPLAY = true;
                        // send YUV converted to 565 2bpp for now...
                        netconn_write(conn, http_bitmap_hdr, sizeof(http_bitmap_hdr) - 1, NETCONN_NOCOPY);
                        char *bmp = bmp_create_header565(camera_get_fb_width(), camera_get_fb_height());
                        err = netconn_write(conn, bmp, sizeof(bitmap565), NETCONN_COPY);
                        free(bmp);
                    } else {
                        printf("b\n");
                        char outstr[120];
                        get_image_mime_info_str(outstr);
                        netconn_write(conn, outstr, sizeof(outstr) - 1, NETCONN_NOCOPY);
                        printf("c\n");
                        //netconn_write(conn, http_yuv422_hdr, sizeof(http_yuv422_hdr) - 1, NETCONN_NOCOPY);
                    }

                //other
                } else {
                printf("4\n");
                    char outstr[120];
                    get_image_mime_info_str(outstr);
                    netconn_write(conn, outstr, sizeof(outstr) - 1, NETCONN_NOCOPY);
                }

                // handle non streaming images (http../get and http:../bmp )
                ESP_LOGD(TAG, "Image requested.");
                //ESP_LOGI(TAG, "task stack: %d", uxTaskGetStackHighWaterMark(NULL));
                bool s_moviemode = is_moviemode_on();
                set_moviemode(false);
                set_moviemode(s_moviemode);
                //ESP_LOGI(TAG, "task stack: %d", uxTaskGetStackHighWaterMark(NULL));
                if (err != ESP_OK) {
                    ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
                } else {
                    ESP_LOGD(TAG, "Done");
                    //Send jpeg
                    if ((s_pixel_format == CAMERA_PF_RGB565) || (s_pixel_format == CAMERA_PF_YUV422)) {
                        ESP_LOGD(TAG, "Converting framebuffer to RGB565 requested, sending...");
                        uint8_t s_line[160*2];
                        uint32_t *fbl;
                        for (int i = 0; i < 120; i++) {
                            printf("sending %d\n", i);
                            fbl = &currFbPtr[(i*160)/2];  //(i*(320*2)/4); // 4 bytes for each 2 pixel / 2 byte read..
                            convert_fb32bit_line_to_bmp565(fbl, s_line,s_pixel_format);
                            err = netconn_write(conn, s_line, 160*2, NETCONN_COPY);
                        }
                        //    ESP_LOGI(TAG, "task stack: %d", uxTaskGetStackHighWaterMark(NULL));
                    } else
                        err = netconn_write(conn, camera_get_fb(), camera_get_data_size(),NETCONN_NOCOPY);
                } // handle .bmp and std gets...
            }// end GET request:
        set_moviemode(s_moviemode);
        }
    }
    netconn_close(conn); /* Close the connection (server closes in HTTP) */
    netbuf_delete(inbuf);/* Delete the buffer (netconn_recv gives us ownership,so we have to make sure to deallocate the buffer) */
}

static void http_server(void *pvParameters)
{
    struct netconn *conn, *newconn;
    err_t err,ert;
    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, NULL, 80);
    netconn_listen(conn);
    do {
        err = netconn_accept(conn, &newconn);
        if (err == ERR_OK) {
            ert = camera_run();
            printf("%s\n",ert == ERR_OK ? "camer run success" : "camera run failed" );
            vTaskDelay(3000 / portTICK_RATE_MS);
            http_server_netconn_serve(newconn);
            netconn_delete(newconn);
        }
    } while (err == ERR_OK);
    netconn_close(conn);
    netconn_delete(conn);
}

void app_main()
{

    ESP_LOGI(TAG,"get free size of 32BIT heap : %d\n",heap_caps_get_free_size(MALLOC_CAP_32BIT));
    currFbPtr = heap_caps_malloc(160*120*2, MALLOC_CAP_32BIT);

    ESP_LOGI(TAG,"%s\n",currFbPtr == NULL ? "currFbPtr is NULL" : "currFbPtr not NULL" );

    ESP_LOGI(TAG,"Starting nvs_flash_init ...");
    nvs_flash_init();

    vTaskDelay(3000 / portTICK_RATE_MS);
    ESP_LOGI(TAG, "Free heap: %u", xPortGetFreeHeapSize());
    ESP_LOGI(TAG, "Wifi Initialized...");

    init_wifi();
    ESP_LOGI(TAG,"get free size of 32BIT heap : %d\n",heap_caps_get_free_size(MALLOC_CAP_32BIT));

    // VERY UNSTABLE without this delay after init'ing wifi...
    // however, much more stable with a new Power Supply
    vTaskDelay(5000 / portTICK_RATE_MS);
    ESP_LOGI(TAG, "Free heap: %u", xPortGetFreeHeapSize());

    // camera init
    esp_err_t err = camera_probe(&config, &camera_model);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera probe failed with error 0x%x", err);
        return;
    }
    if (camera_model == CAMERA_OV7725) {
        ESP_LOGI(TAG, "Detected OV7725 camera, using grayscale bitmap format");
        s_pixel_format = CAMERA_PIXEL_FORMAT;
        config.frame_size = CAMERA_FRAME_SIZE;
    } else if (camera_model == CAMERA_OV7670) {
        ESP_LOGI(TAG, "Detected OV7670 camera");
        s_pixel_format = CAMERA_PIXEL_FORMAT;
        config.frame_size = CAMERA_FRAME_SIZE;
    } else if (camera_model == CAMERA_OV2640) {
        ESP_LOGI(TAG, "Detected OV2640 camera, using JPEG format");
        s_pixel_format = CAMERA_PF_JPEG;
        config.frame_size = CAMERA_FS_VGA;
        config.jpeg_quality = 15;
    } else {
        ESP_LOGE(TAG, "Camera not supported");
        return;
    }

    espilicam_event_group = xEventGroupCreate();
    config.displayBuffer = currFbPtr;
    config.pixel_format = s_pixel_format;

    err = camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }

    vTaskDelay(2000 / portTICK_RATE_MS);

    ESP_LOGD(TAG, "Starting http_server task...");
    // keep an eye on stack... 5784 min with 8048 stck size last count..
    xTaskCreatePinnedToCore(&http_server, "http_server", 4096, NULL, 5, NULL,1);

    ESP_LOGI(TAG, "open http://" IPSTR "/bmp for single image/bitmap image", IP2STR(&s_ip_addr));
    ESP_LOGI(TAG, "open http://" IPSTR "/stream for multipart/x-mixed-replace stream of bitmaps", IP2STR(&s_ip_addr));
    ESP_LOGI(TAG, "open http://" IPSTR "/get for raw image as stored in framebuffer ", IP2STR(&s_ip_addr));

    ESP_LOGI(TAG,"get free size of 32BIT heap : %d\n",heap_caps_get_free_size(MALLOC_CAP_32BIT));
    ESP_LOGI(TAG, "task stack: %d", uxTaskGetStackHighWaterMark(NULL));
    ESP_LOGI(TAG, "Camera demo ready.");

}
