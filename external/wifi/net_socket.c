#include "net_socket.h"
#include "stdio.h"
#include "esp_netif.h"
#include "jeedef.h"
#include "lwip/inet.h"

#define DBG_TAG "net_socket.c"


net_socket net_socket_udp_open(const char *ipaddr, uint16_t port)
{
	int ret, fd, ttl, opt = 0;
	struct sockaddr_in dest_addr;

	dest_addr.sin_family = AF_INET;
	//dest_addr.sin_port = cpu_to_be16(port);

	dest_addr.sin_port = htons(port);
	// dest_addr.sin_addr.s_addr = inet_addr(ipaddr);
	ret = net_dns_resolve(ipaddr, NULL, (uint32_t *)&dest_addr.sin_addr);
	if (ret < 0)
	{
		return -1;
	}

	// LOGI(DBG_TAG, "dest_addr:%s, port:%d", ipaddr, port);

	fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0)
	{
		return fd;
	}
	int enable = 1;
	if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
		printf("setsockopt(SO_REUSEADDR) failed");

	fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);

	ttl = 128;
	ret = setsockopt(fd, IPPROTO_IP, IP_TTL, &ttl, sizeof(ttl));
	if (ret < 0)
	{
		printf("Setsockopt udp IP_MULTICAST_TTL, err!\n");
		goto fail;
	}

	if ((*((uint32_t *)&dest_addr.sin_addr) == 0) || (*((uint32_t *)&dest_addr.sin_addr) == 0xFFFFFFFF))
	{
		opt = 1;
	}

	ret = setsockopt(fd, SOL_SOCKET, SO_BROADCAST, (const void *)&opt, sizeof(int));
	if (ret < 0)
	{
		printf("Setsockopt udp broadcast, err!\n");
		goto fail;
	}

	if (*((uint32_t *)&dest_addr.sin_addr) == 0)
	{
		dest_addr.sin_addr.s_addr = INADDR_ANY;
		ret = bind(fd, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
		if (ret < 0)
		{
			printf("Udp bind erron (%d), %s\n", ret, strerror(errno));
			goto fail;
		}
	}
	printf("udp fd success\r\n");

	uint8_t buf[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10};
	ret = sendto(fd, buf, 10, 0, (struct sockaddr *)&dest_addr, sizeof(struct sockaddr_in));
	if (ret < 0)
	{
		if ((errno != EINTR) && (errno != EAGAIN))
		{
			printf("Udp send errno: %d, %s\n", errno, strerror(errno));
			return NET_SOCKET_FAILED;
		}
	}
	else
	{
		printf("Udp send success ret = %d\n", ret);
	}
	return fd;

fail:
	if (fd > 0)
		close(fd);
	return ret;
}


void net_socket_udp_close(net_socket socket)
{
	if (socket >= 0)
	{
		close(socket);
	}
}

int net_socket_udp_recv(net_socket socket, uint8_t *buf, uint32_t length, uint32_t timeout, char *ipaddr, uint16_t *port)
{
	int ret, len;
	unsigned long dest_addr_len;
	fd_set rfds;
	struct sockaddr_in dest_addr;
	struct timeval tv;

	if ((ipaddr == NULL) || (port == NULL) || (buf == NULL))
	{
		return NET_PARAM_ERRON;
	}

	FD_ZERO(&rfds);
	FD_SET(socket, &rfds);
	tv.tv_sec = timeout / 1000;
	timeout = timeout % 1000;
	tv.tv_usec = timeout * 1000;
	ret = select(socket + 1, &rfds, NULL, NULL, &tv);
	if (ret < 0)
	{
		return ret;
	}
	if (!(ret > 0 && FD_ISSET(socket, &rfds)))
	{
		return NET_SOCKET_WAITING;
	}

	dest_addr_len = sizeof(struct sockaddr_in);
	len = recvfrom(socket, buf, length, 0, (struct sockaddr *)&dest_addr, (socklen_t *)&dest_addr_len);
	if (len < 0)
	{
		if ((errno != EINTR) && (errno != EAGAIN))
		{
			printf("Udp recvfrom errno: %d, %s\n", errno, strerror(errno));
			return NET_SOCKET_FAILED;
		}
	}
	else
	{
		printf("Udp recvfrom success len = %d\n", len);
	}

	// inet_ntoa(dest_addr.sin_addr.s_addr)

	

	

	LOGI("print recv ipaddr", "=========[IP:%s]=======\n", net_ipv4_ntoa(dest_addr.sin_addr.s_addr));


	// memcpy((void *)&ipaddr[0], (void *)&dest_addr.sin_addr.s_addr, 4);
	//*port = ntohs(dest_addr.sin_port);

	return len;
}

int net_socket_udp_send(net_socket socket, uint8_t *buf, uint32_t length, const char *ipaddr, uint16_t port)
{
	int ret, dest_addr_len;
	struct sockaddr_in dest_addr;

	memcpy((void *)&dest_addr.sin_addr.s_addr, (void *)&ipaddr[0], 4);

	dest_addr.sin_port = htons(port);
	dest_addr.sin_family = AF_INET;

	dest_addr_len = sizeof(struct sockaddr_in);

	ret = sendto(socket, buf, length, 0, (struct sockaddr *)&dest_addr, dest_addr_len);
	if (ret < 0)
	{
		if ((errno != EINTR) && (errno != EAGAIN))
		{
			printf("Udp send errno: %d, %s\n", errno, strerror(errno));
			return NET_SOCKET_FAILED;
		}
	}

	return ret;
}


static char inet_buff[18];
const char *net_ipv4_ntoa(long ipaddr)
{
	unsigned char *bytes = (unsigned char *)&ipaddr;

	sprintf(inet_buff, "%d.%d.%d.%d", bytes[0], bytes[1], bytes[2], bytes[3]);
	return inet_buff;
}

int net_dns_resolve(const char *hostname, char *ipaddr, uint32_t *naddr)
{
	struct in_addr sin_addr;
	long nIpaddr = 0;

	if (!inet_aton(hostname, &sin_addr))
	{
		struct addrinfo *ai, *cur;
		struct addrinfo hints;

		memset(&hints, 0, sizeof(hints));
		hints.ai_family = AF_INET;
		if (getaddrinfo(hostname, NULL, &hints, &ai))
		{
			return -1;
		}
		/* getaddrinfo returns a linked list of addrinfo structs.
		 * Even if we set ai_family = AF_INET above, make sure
		 * that the returned one actually is of the correct type. */
		for (cur = ai; cur; cur = cur->ai_next)
		{
			if (cur->ai_family == AF_INET)
			{
				//				*sin_addr = ((struct sockaddr_in *) cur->ai_addr)->sin_addr;
				if (naddr != NULL)
				{
					memcpy((void *)naddr, (void *)&((struct sockaddr_in *)cur->ai_addr)->sin_addr, 4);
				}
				if (ipaddr != NULL)
				{
					memcpy((void *)&nIpaddr, (void *)&sin_addr, 4);
					strcpy(ipaddr, net_ipv4_ntoa(nIpaddr));
				}
				freeaddrinfo(ai);
				return 0;
			}
		}
		freeaddrinfo(ai);
		return -1;
	}
	else
	{
		if (naddr != NULL)
		{
			memcpy((void *)naddr, (void *)&sin_addr, 4);
		}
		if (ipaddr != NULL)
		{
			memcpy((void *)&nIpaddr, (void *)&sin_addr, 4);
			strcpy(ipaddr, net_ipv4_ntoa(nIpaddr));
		}
	}
	return 0;
}

#define MAXSIZE 256
uint8_t recvTcpBuf[MAXSIZE];
net_socket_struct tcpSocket;
net_socket_struct *net_socket_tcp_init(void)
{
	tcpSocket.fd = -1;
	tcpSocket.rx_total = MAXSIZE;
	tcpSocket.tx_length = 0;
	tcpSocket.rx_length = 0;
	tcpSocket.rx_buf = recvTcpBuf;
	tcpSocket.RxWrite = 0;
	tcpSocket.RxRead = 0;
	return &tcpSocket;
}

net_socket_struct *get_net_socket_tcp(void)
{
	return &tcpSocket;
}
