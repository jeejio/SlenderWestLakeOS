#ifndef __NET_SOCKET_H
#define __NET_SOCKET_H

#include "stdint.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define CONNECTING 1
#define CONNECTED 2

    typedef int (*handler)(long arg0, int flags, char *data, int size);

    typedef struct
    {
        int fd;
        uint8_t state;
        uint16_t rx_total;
        uint16_t tx_length;
        uint16_t rx_length;
        uint8_t *tx_buf;
        uint8_t *rx_buf;

        uint16_t RxWrite; /* 接收缓冲区写指针 */
        uint16_t RxRead;  /* 接收缓冲区读指针 */
        handler handler_func;
    } net_socket_struct;

    typedef int net_socket;

    typedef enum
    {
        NET_SOCKET_BREAK = -3,
        NET_PARAM_ERRON = -2,   // param erron
        NET_SOCKET_FAILED = -1, // failed
        NET_SOCKET_WAITING = 0, // waiting
        NET_SOCKET_RECV = 1,
        NET_SOCKET_SEND = 2,

    } NET_SOCKET_STATE;

    const char *net_ipv4_ntoa(long ipaddr);
    int net_dns_resolve(const char *hostname, char *ipaddr, uint32_t *naddr);
    net_socket net_socket_udp_open(const char *ipaddr, uint16_t port);
    void net_socket_udp_close(net_socket socket);
    int net_socket_udp_recv(net_socket socket, uint8_t *buf, uint32_t length, uint32_t timeout, char *ipaddr, uint16_t *port);
    int net_socket_udp_send(net_socket socket, uint8_t *buf, uint32_t length, const char *ipaddr, uint16_t port);


#ifdef __cplusplus
}
#endif

#endif
