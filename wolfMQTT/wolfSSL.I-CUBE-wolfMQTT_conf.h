/**
  ******************************************************************************
  * File Name          : wolfSSL.I-CUBE-wolfMQTT_conf.h
  * Description        : This file provides code for the configuration
  *                      of the wolfSSL.I-CUBE-wolfMQTT_conf.h instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WOLFSSL_I_CUBE_WOLFMQTT_CONF_H__
#define __WOLFSSL_I_CUBE_WOLFMQTT_CONF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/**
	MiddleWare name : wolfSSL.I-CUBE-wolfMQTT.1.19.2
	MiddleWare fileName : ./wolfSSL.I-CUBE-wolfMQTT_conf.h
	MiddleWare version :
*/
/*---------- WOLFMQTT_CONF_FREERTOS -----------*/
#define WOLFMQTT_CONF_FREERTOS      1

/*---------- WOLFMQTT_CONF_TLS -----------*/
#define WOLFMQTT_CONF_TLS      1

/*---------- WOLFMQTT_CONF_V5 -----------*/
#define WOLFMQTT_CONF_V5      1

/*---------- WOLFMQTT_CONF_TIMEOUT -----------*/
#define WOLFMQTT_CONF_TIMEOUT      0

/*---------- WOLFMQTT_CONF_MULTITHREAD -----------*/
#define WOLFMQTT_CONF_MULTITHREAD      0

/*---------- WOLFMQTT_CONF_DEBUG -----------*/
#define WOLFMQTT_CONF_DEBUG      0

/*---------- WOLFMQTT_CONF_IO -----------*/
#define WOLFMQTT_CONF_IO      1

/* ------------------------------------------------------------------------- */
/* Platform */
/* ------------------------------------------------------------------------- */
#define WOLFMQTT_STM32_CUBEMX
#define NO_FILESYSTEM
#define WOLFMQTT_NO_STDIN_CAP

/* ------------------------------------------------------------------------- */
/* Operating System */
/* ------------------------------------------------------------------------- */
#if defined(WOLFMQTT_CONF_FREERTOS) && WOLFMQTT_CONF_FREERTOS == 1
    #define FREERTOS
#endif

/* ------------------------------------------------------------------------- */
/* Enable/Disable Features */
/* ------------------------------------------------------------------------- */
/* TLS */
#undef ENABLE_MQTT_TLS
#if defined(WOLFMQTT_CONF_TLS) && WOLFMQTT_CONF_TLS == 1
	#define ENABLE_MQTT_TLS
#endif

/* MQTT v5 */
#undef WOLFMQTT_V5
#if defined(WOLFMQTT_CONF_V5) && WOLFMQTT_CONF_V5 == 1
	#define WOLFMQTT_V5
#endif

/* TIMEOUT */
#undef WOLFMQTT_NO_TIMEOUT
#if !defined(WOLFMQTT_CONF_TIMEOUT) || WOLFMQTT_CONF_TIMEOUT == 0
	#define WOLFMQTT_NO_TIMEOUT
#endif

/* THREADING */
#undef WOLFMQTT_MULTITHREAD
#if defined(WOLFMQTT_CONF_MULTITHREAD) && WOLFMQTT_CONF_MULTITHREAD == 1
    #ifdef FREERTOS
        #define WOLFMQTT_MULTITHREAD
    #else
        #error "FREERTOS required to enable multi-threading"
    #endif
#endif

/* ------------------------------------------------------------------------- */
/* wolfMQTT IO */
/* ------------------------------------------------------------------------- */
#if defined(WOLFMQTT_CONF_IO) && WOLFMQTT_CONF_IO == 2
    #define WOLFSSL_LWIP
#else
    #define WOLFMQTT_USER_IO
#endif

/* ------------------------------------------------------------------------- */
/* Debugging */
/* ------------------------------------------------------------------------- */
#if defined(WOLFMQTT_CONF_DEBUG) && WOLFMQTT_CONF_DEBUG == 1
    #define DEBUG_WOLFMQTT
#endif

typedef unsigned int size_t;

#define NO_MAIN_DRIVER

#ifdef WOLFSSL_LWIP
    #define HAVE_SOCKET
#endif

#ifdef __cplusplus
}
#endif
#endif /* WOLFSSL_I_CUBE_WOLFMQTT_CONF_H_H */

/**
  * @}
  */

/*****END OF FILE****/
