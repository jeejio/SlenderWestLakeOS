#include "rc522.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/device.h"
#include "drv_spi_master_iomodel.h"
#include "driver/spi_master.h"
#include "hal_spi.h"
#include "hal_gpio.h"

#define RFID_CS_GPIO (9)
#define RFID_RST_GPIO (20)
#define RFID_MISO_GPIO (3)
#define RFID_MOSI_GPIO (8)
#define RFID_SCLK_GPIO (2)

#define SET_SPI_CS                       \
    {                                    \
        gpio_set_level(RFID_CS_GPIO, 1); \
    }
#define CLR_SPI_CS                       \
    {                                    \
        gpio_set_level(RFID_CS_GPIO, 0); \
    }
#define SET_RC522RST                      \
    {                                     \
        gpio_set_level(RFID_RST_GPIO, 1); \
    }
#define CLR_RC522RST                      \
    {                                     \
        gpio_set_level(RFID_RST_GPIO, 0); \
    }

#define SPI_DEV_NAME "spi20"

uint8_t RFID_DATA_BUF[32], MLastSelectedSnr[4];
// uint8_t g_RFID_PSW[6] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36};
// const uint8_t RFID_PASSWORD_A[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t g_RFID_PSW[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint8_t RFID_ID_CODE[4];
uint8_t RFID_DATA_BUF0[16] = {0};
uint8_t XM[16];
uint16_t RFID_Block_Num;
uint8_t MLastSelectedSnr[4]; // 上一次选中的卡

char PcdReset(void);
void CalulateCRC(uint8_t *pIn, uint8_t len, uint8_t *pOut);
char M500PcdConfigISOType(uint8_t type);
uint8_t ReadRawRC(uint8_t Address);
void WriteRawRC(uint8_t Address, uint8_t value);
void SetBitMask(uint8_t reg, uint8_t mask);
void ClearBitMask(uint8_t reg, uint8_t mask);
void PcdAntennaOn(void);
void PcdAntennaOff(void);
char PcdAnticoll(uint8_t *pSnr);
char PcdSelect(uint8_t *pSnr);
char PcdRequest(uint8_t req_code, uint8_t *pTagType);
char PcdAuthState(uint8_t auth_mode, uint8_t addr, uint8_t *pKey, uint8_t *pSnr);
char PcdComMF522(uint8_t Command, uint8_t *pIn, uint8_t InLenByte, uint8_t *pOut, uint8_t *pOutLenBit);
char PcdRead(uint8_t addr, uint8_t *p);
void RFID_Get_ID(uint8_t *pbuf);
uint8_t Read_Block(uint8_t block, uint8_t *pbuf);
uint8_t Write_Block(uint8_t block, uint8_t *pbuf);

void delay_ms(uint32_t ms)
{
    vTaskDelay(ms);
}

void rc522_gpio_config(void)
{
    /************************以下是：esp接口-方式*****************************/
    /*
     gpio_config_t io_conf = {};
     io_conf.intr_type = GPIO_INTR_DISABLE;
     // set as output mode
     io_conf.mode = GPIO_MODE_OUTPUT;
     // bit mask of the pins that you want to set,e.g.GPIO18/19
     io_conf.pin_bit_mask = ((1 << RFID_RST_GPIO) | (1 << RFID_CS_GPIO));
     // disable pull-down mode
     io_conf.pull_down_en = 0;
     // disable pull-up mode
     io_conf.pull_up_en = 0;
     // configure GPIO with the given settings
     gpio_config(&io_conf);
     */
    /************************以下是：jee-iomodel-方式*******************************/
    jee_pin_mode(RFID_RST_GPIO, PIN_MODE_OUTPUT);
    jee_pin_mode(RFID_CS_GPIO, PIN_MODE_OUTPUT);
}

#define RC522_SPI_BUS_NAME "spi2"
#define RC522_SPI_DEV_NAME "spi20"

static spi_device_handle_t spi;
static struct jee_spi_message send_msg;
static struct jee_spi_device *rc522_master_spi;

void spi_init(void)
{
    esp_err_t ret;
    /************************以下是：esp接口-方式*****************************/
    /*
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16 * 320 * 2 + 8 //以字节为单位
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // Clock out at 10 MHz
        .mode = 0,                          // SPI mode 0
        .spics_io_num = -1,//,         // CS pin
        .queue_size = 7,                    // We want to be able to queue 7 transactions at a time
        //.pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    // Initialize the SPI bus
    ret = spi_bus_initialize(SPI_HOST_NUM, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    // Attach the rc522 to the SPI bus
    ret = spi_bus_add_device(SPI_HOST_NUM, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    */
    /************************以下是：jee-iomodel-方式*******************************/
    esp_spi_bus_attach_device(RC522_SPI_BUS_NAME, RC522_SPI_DEV_NAME, RFID_CS_GPIO);
    rc522_master_spi = (struct jee_spi_device *)jee_device_find(SPI_DEV_NAME);
    if (!rc522_master_spi)
    {
        printf("spi sample run failed! can't find spi20 device!\n");
    }
    else
    {
        printf("--------------------> configure rc522 spi device\n");
        static spi_bus_device_configuration_t dev_bus_cfg;
        static spi_bus_config_t bus_cfg = SPI_BUS_TEST_DEFAULT_CONFIG();
        static spi_device_interface_config_t dev_cfg = SPI_DEVICE_TEST_DEFAULT_CONFIG();
        dev_cfg.clock_speed_hz = 20*1000*1000;
        bus_cfg.miso_io_num = RFID_MISO_GPIO;
        bus_cfg.mosi_io_num = RFID_MOSI_GPIO;
        bus_cfg.sclk_io_num = RFID_SCLK_GPIO;
        dev_bus_cfg.spi_bus = &bus_cfg;
        dev_bus_cfg.spi_device = &dev_cfg;
        
        rc522_master_spi->config.pConfig = (void *)(&dev_bus_cfg);
    }
}

uint8_t spi_write_byte(uint8_t data)
{
    esp_err_t ret;
    /************************以下是：esp接口-方式*****************************/
    /*
    spi_transaction_t t;
    ret = spi_device_acquire_bus(spi, portMAX_DELAY);
    if (ret != ESP_OK)
        return ret;

    memset(&t, 0, sizeof(t)); // Zero out the transaction
    t.length = 8;             // data is 8 bits
    t.tx_buffer = &data;
    ret = spi_device_polling_transmit(spi, &t); // Transmit!
    assert(ret == ESP_OK);
    spi_device_release_bus(spi);
    */
    /************************以下是：jee-iomodel-方式*******************************/
    send_msg.send_buf = &data;
    send_msg.recv_buf = NULL;
    send_msg.length = 1;
    send_msg.cs_take = 1;
    send_msg.cs_release = 0;
    send_msg.next = NULL;
    jee_spi_transfer_message(rc522_master_spi, &send_msg);

    return 0;
}

uint8_t spi_read_byte(void)
{
    esp_err_t ret;
    /************************以下是：esp接口-方式*****************************/
    /*
    spi_transaction_t t;
    ret = spi_device_acquire_bus(spi, portMAX_DELAY);
    if (ret != ESP_OK)
        return ret;

    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.flags = SPI_TRANS_USE_RXDATA;
    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
    spi_device_release_bus(spi);
    */
    /************************以下是：jee-iomodel-方式*******************************/
    uint8_t data = 0;
    send_msg.send_buf = NULL;
    send_msg.recv_buf = &data;
    send_msg.length = 1;
    send_msg.cs_take = 1;
    send_msg.cs_release = 0;
    send_msg.next = NULL;
    jee_spi_transfer_message(rc522_master_spi, &send_msg);
    return data;
}
/*******************************************************************
 * 功    能：读RC632寄存器
 * 参数说明：Address[IN]:寄存器地址
 * 返    回：读出的值
 *******************************************************************/
uint8_t ReadRawRC(uint8_t Address)
{
    uint8_t ucAddr;
    uint8_t ucResult = 0;
    CLR_SPI_CS;
    ucAddr = ((Address << 1) & 0x7E) | 0x80;
    spi_write_byte(ucAddr);
    ucResult = spi_read_byte();
    SET_SPI_CS;
    return ucResult;
}

/*******************************************************************
 * 功    能：写RC632寄存器
 * 参数说明：Address[IN]:寄存器地址
 *           value[IN]:写入的值
 *******************************************************************/
void WriteRawRC(uint8_t Address, uint8_t value)
{
    uint8_t ucAddr;

    CLR_SPI_CS;
    ucAddr = ((Address << 1) & 0x7E);
    spi_write_byte(ucAddr);
    spi_write_byte(value);
    SET_SPI_CS;
}

/*******************************************************************
 * 功    能：置RC522寄存器位
 * 参数说明：reg[IN]:寄存器地址
 *           mask[IN]:置位值
 *******************************************************************/
void SetBitMask(uint8_t reg, uint8_t mask)
{
    char tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg, tmp | mask); // set bit mask
}

/*******************************************************************
 * 功    能：清RC522寄存器位
 * 参数说明：reg[IN]:寄存器地址
 *           mask[IN]:清位值
 *******************************************************************/
void ClearBitMask(uint8_t reg, uint8_t mask)
{
    char tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg, tmp & ~mask); // clear bit mask
}

char PcdReset(void)
{
    SET_RC522RST;
    delay_ms(10);
    CLR_RC522RST;
    delay_ms(10);
    SET_RC522RST;
    delay_ms(10);
    WriteRawRC(CommandReg, PCD_RESETPHASE); // 此处一定要两个，不然不读取数据
    WriteRawRC(CommandReg, PCD_RESETPHASE);
    delay_ms(10);
    WriteRawRC(ModeReg, 0x3D); // 和Mifare卡通讯，CRC初始值0x6363
    WriteRawRC(TReloadRegL, 30);
    WriteRawRC(TReloadRegH, 0);
    WriteRawRC(TModeReg, 0x8D);
    WriteRawRC(TPrescalerReg, 0x3E);
    WriteRawRC(TxAutoReg, 0x40); // 必须要
    return MI_OK;
}

/*******************************************************************
 * 关闭天线
 *******************************************************************/
void PcdAntennaOff(void)
{
    ClearBitMask(TxControlReg, 0x03);
}

/*******************************************************************
 * 开启天线
 * 每次启动或关闭天线发射之间应至少有1ms的间隔
 *******************************************************************/
void PcdAntennaOn(void)
{
    uint8_t i;
    i = ReadRawRC(TxControlReg);
    if (!(i & 0x03))
    {
        SetBitMask(TxControlReg, 0x03);
    }
}

void Reset_RC522(void)
{
    PcdReset();      // 功    能：复位RC522
    PcdAntennaOff(); // 关闭天线
    PcdAntennaOn();  // 开启天线
}

/*******************************************************************
 * 设置RC632的工作方式
 *******************************************************************/
char M500PcdConfigISOType(uint8_t type)
{
    if (type == 'A') // ISO14443_A
    {
        ClearBitMask(Status2Reg, 0x08);
        WriteRawRC(ModeReg, 0x3D);   // 3F
        WriteRawRC(RxSelReg, 0x86);  // 84
        WriteRawRC(RFCfgReg, 0x7F);  // 4F
        WriteRawRC(TReloadRegL, 30); // tmoLength);// TReloadVal = 'h6a =tmoLength(dec)
        WriteRawRC(TReloadRegH, 0);
        WriteRawRC(TModeReg, 0x8D);
        WriteRawRC(TPrescalerReg, 0x3E);
        delay_ms(10);
        PcdAntennaOn();
    }
    else
    {
        return 1;
    }

    return MI_OK;
}

static void vRc522InitTask(void *arg)
{
    rc522_gpio_config();
    spi_init();
    Reset_RC522();
    M500PcdConfigISOType('A');
    vTaskDelete(NULL);
}

void RCC522_Init(void)
{
    xTaskCreate(vRc522InitTask, "vRc522InitTask", 2048, NULL, 10, NULL);
}

/*******************************************************************
 * 功    能：寻卡
 * 参数说明: req_code[IN]:寻卡方式
 *                 0x52 = 寻感应区内所有符合14443A标准的卡
 *                 0x26 = 寻未进入休眠状态的卡
 *           pTagType[OUT]：卡片类型代码
 *                 0x4400 = Mifare_UltraLight
 *                 0x0400 = Mifare_One(S50)
 *                 0x0200 = Mifare_One(S70)
 *                 0x0800 = Mifare_Pro(X)
 *                 0x4403 = Mifare_DESFire
 * 返    回: 成功返回MI_OK
 *******************************************************************/
char PcdRequest(uint8_t req_code, uint8_t *pTagType)
{
    char status;
    uint8_t unLen;
    uint8_t ucComMF522Buf[MAXRLEN];

    ClearBitMask(Status2Reg, 0x08);
    WriteRawRC(BitFramingReg, 0x07);
    SetBitMask(TxControlReg, 0x03);

    ucComMF522Buf[0] = req_code;

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 1, ucComMF522Buf, &unLen);

    if ((status == MI_OK) && (unLen == 0x10))
    {
        *pTagType = ucComMF522Buf[0];
        *(pTagType + 1) = ucComMF522Buf[1];
    }
    else
    {
        status = MI_ERR;
    }

    return status;
}

/*******************************************************************
 * 功    能：防冲撞
 * 参数说明: pSnr[OUT]:卡片序列号，4字节
 * 返    回: 成功返回MI_OK
 *******************************************************************/
char PcdAnticoll(uint8_t *pSnr)
{
    char status;
    uint8_t i, snr_check = 0;
    uint8_t unLen;
    uint8_t ucComMF522Buf[MAXRLEN];

    ClearBitMask(Status2Reg, 0x08);
    WriteRawRC(BitFramingReg, 0x00);
    ClearBitMask(CollReg, 0x80);

    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x20;

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 2, ucComMF522Buf, &unLen);

    if (status == MI_OK)
    {
        for (i = 0; i < 4; i++)
        {
            *(pSnr + i) = ucComMF522Buf[i];
            snr_check ^= ucComMF522Buf[i];
        }
        if (snr_check != ucComMF522Buf[i])
        {
            status = MI_ERR;
        }
    }

    SetBitMask(CollReg, 0x80);
    return status;
}

/*******************************************************************
* 功    能：选定卡片
//参数说明: pSnr[IN]:卡片序列号，4字节
//返    回: 成功返回MI_OK
*******************************************************************/
char PcdSelect(uint8_t *pSnr)
{
    char status;
    uint8_t i;
    uint8_t unLen;
    uint8_t ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i = 0; i < 4; i++)
    {
        ucComMF522Buf[i + 2] = *(pSnr + i);
        ucComMF522Buf[6] ^= *(pSnr + i);
    }
    CalulateCRC(ucComMF522Buf, 7, &ucComMF522Buf[7]);

    ClearBitMask(Status2Reg, 0x08);

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 9, ucComMF522Buf, &unLen);

    if ((status == MI_OK) && (unLen == 0x18))
    {
        status = MI_OK;
    }
    else
    {
        status = MI_ERR;
    }

    return status;
}

/*******************************************************************
 * 功    能：验证卡片密码
 * 参数说明: auth_mode[IN]: 密码验证模式
 *                  0x60 = 验证A密钥
 *                  0x61 = 验证B密钥
 *           addr[IN]：块地址
 *           pKey[IN]：密码
 *           pSnr[IN]：卡片序列号，4字节
 * 返    回: 成功返回MI_OK
 *******************************************************************/
char PcdAuthState(uint8_t auth_mode, uint8_t addr, uint8_t *pKey, uint8_t *pSnr)
{
    char status;
    uint8_t unLen;
    //     uint8_t   i;
    uint8_t ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = auth_mode;
    ucComMF522Buf[1] = addr;
    //    for (i=0; i<6; i++)
    //    {    ucComMF522Buf[i+2] = *(pKey+i);   }
    //    for (i=0; i<6; i++)
    //    {    ucComMF522Buf[i+8] = *(pSnr+i);   }
    memcpy(&ucComMF522Buf[2], pKey, 6);
    memcpy(&ucComMF522Buf[8], pSnr, 4);

    status = PcdComMF522(PCD_AUTHENT, ucComMF522Buf, 12, ucComMF522Buf, &unLen);
    if ((status != MI_OK) || (!(ReadRawRC(Status2Reg) & 0x08)))
    {
        status = MI_ERR;
    }

    return status;
}

/********************************************************************
 * 功    能：读取M1卡一块数据
 * 参数说明: addr[IN]：块地址
 *           p [OUT]：读出的数据，16字节
 * 返    回: 成功返回MI_OK
 ********************************************************************/
char PcdRead(uint8_t addr, uint8_t *p)
{
    char status;
    uint8_t unLen;
    uint8_t i, ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_READ;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);
    if ((status == MI_OK) && (unLen == 0x90))
    //   {   memcpy(p , ucComMF522Buf, 16);   }
    {
        for (i = 0; i < 16; i++)
        {
            *(p + i) = ucComMF522Buf[i];
        }
    }
    else
    {
        status = MI_ERR;
    }

    return status;
}

/********************************************************************
 * 功    能：写数据到M1卡一块
 * 参数说明: addr[IN]：块地址
 *           p [IN]：写入的数据，16字节
 * 返    回: 成功返回MI_OK
 ********************************************************************/
char PcdWrite(uint8_t addr, uint8_t *p)
{
    char status;
    uint8_t unLen;
    uint8_t i, ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_WRITE;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {
        status = MI_ERR;
    }

    if (status == MI_OK)
    {
        // memcpy(ucComMF522Buf, p , 16);
        for (i = 0; i < 16; i++)
        {
            ucComMF522Buf[i] = *(p + i);
        }
        CalulateCRC(ucComMF522Buf, 16, &ucComMF522Buf[16]);

        status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 18, ucComMF522Buf, &unLen);
        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {
            status = MI_ERR;
        }
    }

    return status;
}

/********************************************************************
 * 功    能：命令卡片进入休眠状态
 * 返    回: 成功返回MI_OK
 ********************************************************************/
char PcdHalt(void)
{
    uint8_t status;
    uint8_t unLen;
    uint8_t ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_HALT;
    ucComMF522Buf[1] = 0;
    CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);
    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);
    // return MI_OK;
    return status;
}

/********************************************************************
 * 用MF522计算CRC16函数
 ********************************************************************/
void CalulateCRC(uint8_t *pIn, uint8_t len, uint8_t *pOut)
{
    uint8_t i, n;
    ClearBitMask(DivIrqReg, 0x04);
    WriteRawRC(CommandReg, PCD_IDLE);
    SetBitMask(FIFOLevelReg, 0x80);
    for (i = 0; i < len; i++)
    {
        WriteRawRC(FIFODataReg, *(pIn + i));
    }
    WriteRawRC(CommandReg, PCD_CALCCRC);
    i = 0xFF;
    do
    {
        n = ReadRawRC(DivIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x04));
    pOut[0] = ReadRawRC(CRCResultRegL);
    pOut[1] = ReadRawRC(CRCResultRegM);
}

/********************************************************************
 * 功    能：通过RC522和ISO14443卡通讯
 * 参数说明：Command[IN]:RC522命令字
 *           pIn [IN]:通过RC522发送到卡片的数据
 *           InLenByte[IN]:发送数据的字节长度
 *           pOut [OUT]:接收到的卡片返回数据
 *           *pOutLenBit[OUT]:返回数据的位长度
 ********************************************************************/
char PcdComMF522(uint8_t Command,
                 uint8_t *pIn,
                 uint8_t InLenByte,
                 uint8_t *pOut,
                 uint8_t *pOutLenBit)
{
    char status = MI_ERR;
    uint8_t irqEn = 0x00;
    uint8_t waitFor = 0x00;
    uint8_t lastBits;
    uint8_t n;
    uint16_t i;
    switch (Command)
    {
    case PCD_AUTHENT:
        irqEn = 0x12;
        waitFor = 0x10;
        break;
    case PCD_TRANSCEIVE:
        irqEn = 0x77;
        waitFor = 0x30;
        break;
    default:
        break;
    }

    WriteRawRC(ComIEnReg, irqEn | 0x80);
    ClearBitMask(ComIrqReg, 0x80); // 清所有中断位
    WriteRawRC(CommandReg, PCD_IDLE);
    SetBitMask(FIFOLevelReg, 0x80); // 清FIFO缓存

    for (i = 0; i < InLenByte; i++)
    {
        WriteRawRC(FIFODataReg, pIn[i]);
    }
    WriteRawRC(CommandReg, Command);
    //   	 n = ReadRawRC(CommandReg);

    if (Command == PCD_TRANSCEIVE)
    {
        SetBitMask(BitFramingReg, 0x80);
    } // 开始传送

    // i = 600;//根据时钟频率调整，操作M1卡最大等待时间25ms
    i = 2000; // 根据时钟频率调整，操作M1卡最大等待时间25ms
              //  	  i = 100000;
    do
    {
        n = ReadRawRC(ComIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitFor));
    ClearBitMask(BitFramingReg, 0x80);

    if (i != 0)
    {
        if (!(ReadRawRC(ErrorReg) & 0x1B))
        {
            status = MI_OK;
            if (n & irqEn & 0x01)
            {
                status = MI_NOTAGERR;
            }
            if (Command == PCD_TRANSCEIVE)
            {
                n = ReadRawRC(FIFOLevelReg);
                lastBits = ReadRawRC(ControlReg) & 0x07;
                if (lastBits)
                {
                    *pOutLenBit = (n - 1) * 8 + lastBits;
                }
                else
                {
                    *pOutLenBit = n * 8;
                }
                if (n == 0)
                {
                    n = 1;
                }
                if (n > MAXRLEN)
                {
                    n = MAXRLEN;
                }
                for (i = 0; i < n; i++)
                {
                    pOut[i] = ReadRawRC(FIFODataReg);
                }
            }
        }
        else
        {
            status = MI_ERR;
        }
    }

    SetBitMask(ControlReg, 0x80); // stop timer now
    WriteRawRC(CommandReg, PCD_IDLE);
    return status;
}

void RFID_Get_ID(uint8_t *pbuf) // 卡号4字节
{
    int8_t status;
    PcdReset();                                          // 复位RC522
    status = PcdRequest(PICC_REQALL, &RFID_DATA_BUF[0]); // 寻天线区内未进入休眠状态的卡，返回卡片类型 2字节
    if (status != MI_OK)
        return;
    status = PcdAnticoll(&RFID_DATA_BUF[2]); // 防冲撞，返回卡的序列号 4字节
    if (status != MI_OK)
        return;
    memcpy(pbuf, &RFID_DATA_BUF[2], 4);   // 拷贝卡号
    status = PcdSelect(MLastSelectedSnr); // 选卡
    if (status != MI_OK)
        return;
    status = PcdAuthState(PICC_AUTHENT1A, 4, (uint8_t *)g_RFID_PSW, MLastSelectedSnr); // 验证A密钥
    if (status != MI_OK)
    {
        return;
    }
    status = PcdRead(4, RFID_DATA_BUF);
    PcdHalt();
    if (status != MI_OK)
    {
        return;
    }
}

uint8_t Compare(uint8_t *x, uint8_t *y, uint8_t len)
{
    uint8_t m;
    for (m = 0; m < len; m++)
    {
        if ((*(x + m)) != (*(y + m)))
        {
            return MI_ERR;
        }
    }
    return MI_OK;
}

uint8_t Read_Block(uint8_t block, uint8_t *pbuf)
{
    char status;
    PcdReset();                                          // 复位RC522
    status = PcdRequest(PICC_REQALL, &RFID_DATA_BUF[0]); // 寻天线区内未进入休眠状态的卡，返回卡片类型 2字节
    if (status != MI_OK)
    {
        return MI_ERR;
    }
    status = PcdAnticoll(&RFID_DATA_BUF[2]); // 防冲撞，返回卡的序列号 4字节
    if (status != MI_OK)
    {
        return MI_ERR;
    }
    memcpy(MLastSelectedSnr, &RFID_DATA_BUF[2], 4); // 从&RevBuffer[2]这个地址开始拷贝4个字节到指针起始位置MLastSelectedSnr
    status = PcdSelect(MLastSelectedSnr);           // 选卡
    if (status != MI_OK)
    {
        return MI_ERR;
    }
    status = PcdAuthState(PICC_AUTHENT1A, block, g_RFID_PSW, MLastSelectedSnr); // 验证密匙
    if (status != MI_OK)
    {
        return MI_ERR;
    }
    /*
    status=PcdWrite(addr,&cardData[0]);//把WriteData[]数组中的数据写到M1卡某一块中
    if(status!=MI_OK)
    {
        return MI_ERR;
    }
    */
    status = PcdRead(block, pbuf); // 从M1卡某一块读取到的数据存放在数组中
    PcdHalt();
    if (status != MI_OK)
    {
        return MI_ERR;
    }
    return MI_OK;
}

uint8_t Write_Block(uint8_t block, uint8_t *pbuf)
{
    char status;

    PcdReset();                                          // 复位RC522
    status = PcdRequest(PICC_REQALL, &RFID_DATA_BUF[0]); // 寻天线区内未进入休眠状态的卡，返回卡片类型 2字节
    if (status != MI_OK)
    {
        return MI_ERR;
    }
    status = PcdAnticoll(&RFID_DATA_BUF[2]); // 防冲撞，返回卡的序列号 4字节
    if (status != MI_OK)
    {
        return MI_ERR;
    }
    memcpy(MLastSelectedSnr, &RFID_DATA_BUF[2], 4); // 从&RevBuffer[2]这个地址开始拷贝4个字节到指针起始位置MLastSelectedSnr
    status = PcdSelect(MLastSelectedSnr);           // 选卡
    if (status != MI_OK)
    {
        return MI_ERR;
    }
    status = PcdAuthState(PICC_AUTHENT1A, block, g_RFID_PSW, MLastSelectedSnr); // 验证密匙
    if (status != MI_OK)
    {
        return MI_ERR;
    }
    status = PcdWrite(block, pbuf); // 把WriteData[]数组中的数据写到M1卡某一块中
    if (status != MI_OK)
    {
        return MI_ERR;
    }
    status = PcdRead(block, XM); // 从M1卡某一块读取到的数据存放在Read_Data[]数组中
    if (status != MI_OK)
    {
        return MI_ERR;
    }
    while (Compare(pbuf, XM, 8) != MI_OK)
    {
        status = PcdAuthState(PICC_AUTHENT1A, block, g_RFID_PSW, MLastSelectedSnr); // 验证A密匙
        if (status != MI_OK)
        {
            return MI_ERR;
        }
        status = PcdWrite(block, pbuf); // 把WriteData[]数组中的数据写到M1卡某一块中
        if (status != MI_OK)
        {
            return MI_ERR;
        }
        status = PcdRead(block, XM); // 从M1卡某一块读取到的数据存放在Read_Data[]数组中
        if (status != MI_OK)
        {
            return MI_ERR;
        }
    }
    if (PcdHalt() != MI_OK)
    {
        return MI_ERR;
    }
    return MI_OK;
}

void rfid_get_card_id(uint8_t *id)
{
    RFID_Get_ID(id);
    printf("rfid_get_card_id %02x%02x%02x%02x\n", id[0], id[1], id[2], id[3]);
}

void rfid_get_card_info(uint8_t *dat)
{
    uint8_t id[4] = {0, 0, 0, 0};
    RFID_Get_ID(&id);
    memcpy(dat, id, 4);
    Read_Block(2, dat + 4);
    printf("rfid_get_card_info id data %x%x%x%x %s\n", dat[0], dat[1], dat[2], dat[3], dat + 4);
}

void rfid_set_card_data(uint8_t *dat)
{
    uint8_t data[17];
    memset(data, 0, 17);
    memcpy(data, dat, strlen((char *)dat));
    Write_Block(2, data);
    printf("rfid_set_card_data %s\n", data);
}

uint8_t rfid_get_card_id_upload(char *str)
{
#if 1
    uint8_t id[4];
    static uint8_t id2[17];
    uint8_t id3[4] = {0, 0, 0, 0};
    memset(id, 0, 4);
    RFID_Get_ID(&id);
    uint8_t res = 0;
    if (memcmp(id, id3, 4) != 0 && memcmp(id, id2, 4) != 0)
    {
        memcpy(id2, id, 4);
        uint8_t dat[17];
        memset(dat, 0, 17);
        Read_Block(2, dat);
        sprintf(str, "{\"ID\":\"%02x%02x%02x%02x\",\"data\":\"%s\"}", id[0], id[1], id[2], id[3], dat);
        printf("hal_rfid_get_card_id_upload %s\n", str);
        res = 1;
    }
    else if (memcmp(id, id3, 4) == 0)
    {
        memset(id2, 0, 4);
    }
    return res;
#endif
}
