  
#ifndef _DRIVER_ADC_H_
#define _DRIVER_ADC_H_

#ifdef __cplusplus
extern "C" {
#endif

#define WIFI_PASSWORD CONFIG_WIFI_PASSWORD
#define WIFI_SSID     CONFIG_WIFI_SSID
/*
*CAMERA_PF_RGB565 = 0,       //!< RGB, 2 bytes per pixel
*CAMERA_PF_YUV422 = 1,       //!< YUYV, 2 bytes per pixel
*CAMERA_PF_GRAYSCALE = 2,    //!< 1 byte per pixel
*CAMERA_PF_JPEG = 3,         //!< JPEG compressed
*CAMERA_PF_RGB555 = 4,       //!< RGB, 2 bytes per pixel
*CAMERA_PF_RGB444 = 5,       //!< RGB, 2 bytes per pixel
*/
#define CAMERA_PIXEL_FORMAT CAMERA_PF_RGB565

/*
 *CAMERA_FS_QQVGA = 4,     //!< 160x120
 *CAMERA_FS_HQVGA = 7,     //!< 240x160
 *CAMERA_FS_QCIF = 6,      //!< 176x144
 *CAMERA_FS_QVGA = 8,      //!< 320x240
 *CAMERA_FS_VGA = 10,      //!< 640x480
 *CAMERA_FS_SVGA = 11,     //!< 800x600
*/
#define CAMERA_FRAME_SIZE CAMERA_FS_VGA

#ifdef __cplusplus
}
#endif

#endif  /*_DRIVER_ADC_H_*/
