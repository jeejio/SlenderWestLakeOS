#include "md280.h"
#include <stdio.h>
#include "hal_serial.h"
#include "hal_sensor.h"
#include "drv_uart_iomodel.h"
#include "hal/uart_types.h"
#include "hal/gpio_types.h"
#include "uart.h"

#define UART1_NAME "uart1"
jee_device_t uart1Dev = NULL;

#define BUF_SIZE (128)

#define TXD_PIN (GPIO_NUM_3)
#define RXD_PIN (GPIO_NUM_2)
struct uart_plat_data
{
    jee_uint8_t *xBuf;
    jee_uint32_t bufSize;
    jee_int32_t real_size;
};

uint8_t start_set[] = {0x5A, 0x00, 0x00, 0x0a, 0x53, 0x5f, 0x43, 0x4d, 0x44, 0x5f, 0x30, 0x30, 0x30, 0x31, 0x12, 0xA5};
uint8_t end_set[] = {0x5A, 0x00, 0x00, 0x0a, 0x53, 0x5f, 0x43, 0x4d, 0x44, 0x5f, 0x30, 0x30, 0x30, 0x30, 0x13, 0xA5};
uint8_t mode_set[] = {0x5A, 0x00, 0x00, 0x0a, 0x53, 0x5f, 0x43, 0x4d, 0x44, 0x5f, 0x30, 0x32, 0x30, 0x46, 0x67, 0xA5};
uint8_t code_set[] = {0x5A, 0x00, 0x00, 0x0a, 0x53, 0x5f, 0x43, 0x4d, 0x44, 0x5f, 0x4d, 0x53, 0x33, 0x30, 0x0e, 0xA5};
static bool scaner_state = false;
uint8_t *code_upload_str=NULL;
uint8_t code_upload_len = 0;

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    // esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    code_upload_str= (uint8_t *)malloc(BUF_SIZE);

    struct uart_plat_data xUartData = {
        .xBuf = data,
        .bufSize = BUF_SIZE - 1,
        .real_size = 0,
    };

    memset(data, 0, BUF_SIZE);

    memset(code_upload_str, 0, BUF_SIZE - 1);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    memcpy(xUartData.xBuf, start_set, sizeof(start_set));
    xUartData.real_size = sizeof(start_set);
    jee_device_control(uart1Dev, JEE_DEVICE_CTRL_UART_WRITE, (void *)&xUartData);

    vTaskDelay(100 / portTICK_PERIOD_MS);
    memcpy(xUartData.xBuf, mode_set, sizeof(mode_set));
    xUartData.real_size = sizeof(mode_set);
    jee_device_control(uart1Dev, JEE_DEVICE_CTRL_UART_WRITE, (void *)&xUartData);

    vTaskDelay(100 / portTICK_PERIOD_MS);
    memcpy(xUartData.xBuf, code_set, sizeof(code_set));
    xUartData.real_size = sizeof(code_set);
    jee_device_control(uart1Dev, JEE_DEVICE_CTRL_UART_WRITE, (void *)&xUartData);

    vTaskDelay(100 / portTICK_PERIOD_MS);
    memcpy(xUartData.xBuf, end_set, sizeof(end_set));
    xUartData.real_size = sizeof(end_set);
    jee_device_control(uart1Dev, JEE_DEVICE_CTRL_UART_WRITE, (void *)&xUartData);

    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (1)
    {
        jee_device_control(uart1Dev, JEE_DEVICE_CTRL_UART_READ, (void *)&xUartData);

        // const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, pdMS_TO_TICKS(50));
        //printf("xUartData.real_size %ld\n",xUartData.real_size);
        if (xUartData.real_size > 0)
        {
            printf("res %s\n", xUartData.xBuf);

            scaner_state = true;
            memset(code_upload_str, 0, BUF_SIZE - 1);
            code_upload_len = xUartData.real_size - 1;
            memcpy(code_upload_str, xUartData.xBuf, code_upload_len);

            scaner_state = true;
            memset(xUartData.xBuf, 0, BUF_SIZE);
            // data[rxBytes] = 0;
            // ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            // uart_write_bytes(UART_NUM_1, data, rxBytes);
            //  ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(data);
}

void md280Init(void)
{

    uart1Dev = jee_device_find(UART1_NAME);
    if (uart1Dev == JEE_NULL)
    {
        printf("can not find %s Model\n", UART1_NAME);
        return;
    }
    printf("jee_device_find\n");
    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(uart1Dev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open %s device\n", UART1_NAME);
        return;
    }

    #if 0
     printf("jee_device_open\n");
    struct serial_configure pconfig;
    pconfig.baud_rate = 9600;
    pconfig.data_bits = UART_DATA_8_BITS;
    pconfig.stop_bits = UART_STOP_BITS_1;
    pconfig.parity = UART_PARITY_DISABLE;
    pconfig.flowcontrol = UART_HW_FLOWCTRL_DISABLE;
    pconfig.rxPinNo = RXD_PIN;
    pconfig.txPinNo = TXD_PIN;
    pconfig.rtsPinNo = UART_PIN_NO_CHANGE;
    pconfig.ctsPinNo = UART_PIN_NO_CHANGE;
     printf("jee_device_control\n");
    jee_err_t ret = jee_device_control(uart1Dev, JEE_DEVICE_CTRL_CONFIG, (void *)&pconfig);
    if (JEE_EOK != ret)
    {
        printf("uart set err!\n");
    }
    #endif
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, 10, NULL);
}
uint16_t md280GetData(uint8_t *data)
{
    if (scaner_state == true)
    {
        scaner_state = false;
        memcpy(data, code_upload_str, code_upload_len);
        return code_upload_len;
    }
    else
    {
        return 0;
    }
}
