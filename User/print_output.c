#include "print_output.h"

#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include "lwip/netif.h"
#include "lwip/ip_addr.h"

#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef* g_p_debug_uart = NULL;
static osMutexId_t g_uart_mutex_handle;

static const osMutexAttr_t uart_mutex_attributes = {.name = "uartMutex"};

void print_output_init(UART_HandleTypeDef* p_huart) {
    g_p_debug_uart = p_huart;
    g_uart_mutex_handle = osMutexNew(&uart_mutex_attributes);
}

int _write(int file, char* ptr, int len) {
    if (g_p_debug_uart == NULL) { return 0; }

    if (osKernelGetState() == osKernelRunning && g_uart_mutex_handle != NULL) {
        osMutexAcquire(g_uart_mutex_handle, osWaitForever);
    }

    HAL_UART_Transmit(g_p_debug_uart, (uint8_t*)ptr, len, 100);

    if (osKernelGetState() == osKernelRunning && g_uart_mutex_handle != NULL) {
        osMutexRelease(g_uart_mutex_handle);
    }

    return len;
}

void print_output_netif_info(struct netif* netif) {
    if (netif == NULL) {
        printf("netif == NULL\n");
        return;
    }

    char ipbuf[16], gwbuf[16], nmbuf[16];

    ipaddr_ntoa_r(&netif->ip_addr, ipbuf, sizeof(ipbuf));
    ipaddr_ntoa_r(&netif->gw, gwbuf, sizeof(gwbuf));
    ipaddr_ntoa_r(&netif->netmask, nmbuf, sizeof(nmbuf));

    char macbuf[3 * NETIF_MAX_HWADDR_LEN];
    int maclen = netif->hwaddr_len;

    if (maclen > 0 && maclen <= NETIF_MAX_HWADDR_LEN) {
        char* p = macbuf;
        for (int i = 0; i < maclen; ++i) {
            if (i == 0)
                p += sprintf(p, "%02X", netif->hwaddr[i]);
            else
                p += sprintf(p, ":%02X", netif->hwaddr[i]);
        }
    } else {
        strcpy(macbuf, "N/A");
    }

    const char* link = netif_is_link_up(netif) ? "up" : "down";
    const char* up = netif_is_up(netif) ? "yes" : "no";

#if LWIP_DHCP
    const char* dhcp_state = "not used";
    if (netif->dhcp != NULL) { dhcp_state = "client active"; }
#else
    const char* dhcp_state = "disabled";
#endif

    printf(
        "link: %s, netif_is_up: %s, dhcp: %s, netif '%c'\n", link, up, dhcp_state, netif->name[0]);
    printf("ip: %s, netmask: %s, gateway: %s, mac: %s\n", ipbuf, nmbuf, gwbuf, macbuf);
}