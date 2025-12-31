#ifndef PRINT_OUTPUT_H
#define PRINT_OUTPUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "lwip/netif.h"

/**
 * @brief Initialize the Output Interface (Mutex and UART handle)
 *
 * @param[in] p_huart Pointer to the UART handle used for printf
 * @return void
 */
void print_output_init(UART_HandleTypeDef* p_huart);

/**
 * @brief Print information about the network interface
 *
 * @param[in] netif Pointer to the LwIP network interface struct
 * @return void
 */
void print_output_netif_info(struct netif* netif);

#ifdef __cplusplus
}
#endif

#endif /* PRINT_OUTPUT_H */