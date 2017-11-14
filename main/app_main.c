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
#include "freertos/semphr.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/api.h"

#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "soc/spi_reg.h"
#include "soc/gpio_reg.h"
#include "esp_attr.h"

#include "soc/gpio_struct.h"
#include "esp_err.h"
#include "camera.h"
#include "user_config.h"

#include "bitmap.h"

static const char* TAG = "ESP-CAM";

// config camera
static camera_pixelformat_t s_pixel_format = CAMERA_PIXEL_FORMAT;
static camera_framesize_t s_framesize = CAMERA_FRAME_SIZE;
static camera_model_t camera_model;
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
    //.test_pattern_enabled = CONFIG_ENABLE_TEST_PATTERN,
};

//Warning: This gets squeezed into IRAM.
volatile static uint32_t *currFbPtr __attribute__ ((aligned(4))) = NULL;

inline uint8_t unpack(int byteNumber, uint32_t value) {
    return (value >> (byteNumber * 8));
}

// camera code
const static char http_hdr[] = "HTTP/1.1 200 OK\r\n";
const static char http_bitmap_hdr[] ="Content-type: image/bitmap\r\n\r\n";

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;
static ip4_addr_t s_ip_addr;

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
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_set_ps(WIFI_PS_NONE) );
    ESP_LOGI(TAG, "Connecting to \"%s\"", wifi_config.sta.ssid);
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected");
}

static void convert_fb32bit_line_to_bmp565(uint32_t *srcline, uint8_t *destline, const camera_pixelformat_t format) {

    uint16_t pixel565 = 0;
    uint16_t pixel565_2 = 0;
    uint32_t long2px = 0;
    uint16_t *sptr;
    uint16_t conver_times;
    uint16_t current_src_pos = 0, current_dest_pos = 0;

    switch(s_framesize)
    {
        case CAMERA_FS_SVGA:
            conver_times = 800;
            break;
        case CAMERA_FS_VGA:
            conver_times = 640;
            break;
        case CAMERA_FS_QVGA:
            conver_times = 320;
            break;
        case CAMERA_FS_QCIF:
            conver_times = 176;
            break;
        case CAMERA_FS_HQVGA:
            conver_times = 240;
            break;
        case CAMERA_FS_QQVGA:
            conver_times = 160;
            break;
        default:
            printf("framesize not init\n");
            break;
    }

    for ( int current_pixel_pos = 0; current_pixel_pos < conver_times; current_pixel_pos += 2 )
    {
        current_src_pos = current_pixel_pos / 2;
        long2px = srcline[current_src_pos];
       
        pixel565 =  (unpack(2,long2px) << 8) | unpack(3,long2px);
        pixel565_2 = (unpack(0,long2px) << 8) | unpack(1,long2px);

        sptr = &destline[current_dest_pos];
        *sptr = pixel565;
        sptr = &destline[current_dest_pos+2];
        *sptr = pixel565_2;
        current_dest_pos += 4;
        
    }
}

static void http_server_netconn_serve(struct netconn *conn)
{
    struct netbuf *inbuf;
    char *buf;
    u16_t buflen;
    err_t err;
    err = netconn_recv(conn, &inbuf);
    if (err == ERR_OK) {
        netbuf_data(inbuf, (void**) &buf, &buflen);
        if (buflen >= 5 && buf[0] == 'G' && buf[1] == 'E' && buf[2] == 'T' && buf[3] == ' ' && buf[4] == '/') {
            ESP_LOGD(TAG, "Start Image Sending...");
            netconn_write(conn, http_hdr, sizeof(http_hdr) - 1,NETCONN_NOCOPY);
            netconn_write(conn, http_bitmap_hdr, sizeof(http_bitmap_hdr) - 1, NETCONN_NOCOPY);
            if (memcmp(&buf[5], "bmp", 3) == 0) {
                char *bmp = bmp_create_header565(camera_get_fb_width(), camera_get_fb_height());
                err = netconn_write(conn, bmp, sizeof(bitmap565), NETCONN_COPY);
                free(bmp);
            } else {
                char outstr[120];
                get_image_mime_info_str(outstr);
                netconn_write(conn, outstr, sizeof(outstr) - 1, NETCONN_NOCOPY);
            }
            
            if ((s_pixel_format == CAMERA_PF_RGB565) || (s_pixel_format == CAMERA_PF_YUV422)) {
                uint8_t s_line[640*2];
                uint32_t *fbl;
                for (int i = 0; i < 480; i++) {
                    fbl = &currFbPtr[(i*640)/2];  //(i*(320*2)/4); // 4 bytes for each 2 pixel / 2 byte read..
                    convert_fb32bit_line_to_bmp565(fbl, s_line, s_pixel_format);
                    err = netconn_write(conn, s_line, 640*2, NETCONN_COPY);
                }
                ESP_LOGD(TAG,"Image sending Done ...");
            } else{
                err = netconn_write(conn, camera_get_fb(), camera_get_data_size(),NETCONN_NOCOPY);
            }
           
        }// end GET request:
    }
    netconn_close(conn); /* Close the connection (server closes in HTTP) */
    netbuf_delete(inbuf);/* Delete the buffer (netconn_recv gives us ownership,so we have to make sure to deallocate the buffer) */
}

static void http_server(void *pvParameters)
{
    struct netconn *conn, *newconn;  
    err_t err,ert;
    conn = netconn_new(NETCONN_TCP);  /* creat TCP connector */
    netconn_bind(conn, NULL, 80);  /* bind HTTP port */
    netconn_listen(conn);  /* server listen connect */
    do {
        err = netconn_accept(conn, &newconn);
        if (err == ERR_OK) {    /* new conn is coming */
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
    currFbPtr = heap_caps_malloc(640*480*2, MALLOC_CAP_32BIT);

    ESP_LOGI(TAG,"%s\n",currFbPtr == NULL ? "currFbPtr is NULL" : "currFbPtr not NULL" );
    ESP_LOGI(TAG,"framebuffer address is:%p\n",currFbPtr);
    

    ESP_LOGI(TAG,"Starting nvs_flash_init ...");
    nvs_flash_init();

    vTaskDelay(3000 / portTICK_RATE_MS);
    ESP_LOGI(TAG, "Free heap: %u", xPortGetFreeHeapSize());
    ESP_LOGI(TAG, "Wifi Initialized...");

    init_wifi();

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

    if (camera_model == CAMERA_OV7670) {
        ESP_LOGI(TAG, "Detected OV7670 camera");
        s_pixel_format = CAMERA_PIXEL_FORMAT;
        config.frame_size = CAMERA_FRAME_SIZE;
    }else{
        ESP_LOGI(TAG,"Cant detected ov7670 camera");
    }

    config.displayBuffer = currFbPtr;
    config.pixel_format = s_pixel_format;

    err = camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }

    vTaskDelay(2000 / portTICK_RATE_MS);

    ESP_LOGD(TAG, "Starting http_server task...");
    xTaskCreatePinnedToCore(&http_server, "http_server", 4096, NULL, 5, NULL,1);

    ESP_LOGI(TAG, "open http://" IPSTR "/bmp for single image/bitmap image", IP2STR(&s_ip_addr));
    ESP_LOGI(TAG, "open http://" IPSTR "/get for raw image as stored in framebuffer ", IP2STR(&s_ip_addr));

    ESP_LOGI(TAG,"get free size of 32BIT heap : %d\n",heap_caps_get_free_size(MALLOC_CAP_32BIT));
    ESP_LOGI(TAG, "task stack: %d", uxTaskGetStackHighWaterMark(NULL));
    ESP_LOGI(TAG, "Camera demo ready.");
}