#include "paj7620u2.h"
#include <stdio.h>
#include "esp32c3/rom/ets_sys.h" //for ets_delay_us()
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "hal_i2c.h"
#include "hal_sensor.h"
/***************************************************************************/
// #define BIT(x)  1 << x

// REGISTER DESCRIPTION
#define PAJ7620_VAL(val, maskbit) (val << maskbit)
#define PAJ7620_ADDR_BASE 0x00

// REGISTER BANK SELECT
#define PAJ7620_REGITER_BANK_SEL (PAJ7620_ADDR_BASE + 0xEF) // W

// DEVICE ID
#define PAJ7620_ID 0x73

#define GPIO_INT_INPUT 2
#define GPIO_INT_INPUT_PIN_SEL ((1ULL << GPIO_INT_INPUT))

// REGISTER BANK 0
#define PAJ7620_ADDR_SUSPEND_CMD (PAJ7620_ADDR_BASE + 0x3)        // W
#define PAJ7620_ADDR_GES_PS_DET_MASK_0 (PAJ7620_ADDR_BASE + 0x41) // RW
#define PAJ7620_ADDR_GES_PS_DET_MASK_1 (PAJ7620_ADDR_BASE + 0x42) // RW
#define PAJ7620_ADDR_GES_PS_DET_FLAG_0 (PAJ7620_ADDR_BASE + 0x43) // R
#define PAJ7620_ADDR_GES_PS_DET_FLAG_1 (PAJ7620_ADDR_BASE + 0x44) // R
#define PAJ7620_ADDR_STATE_INDICATOR (PAJ7620_ADDR_BASE + 0x45)   // R
#define PAJ7620_ADDR_PS_HIGH_THRESHOLD (PAJ7620_ADDR_BASE + 0x69) // RW
#define PAJ7620_ADDR_PS_LOW_THRESHOLD (PAJ7620_ADDR_BASE + 0x6A)  // RW
#define PAJ7620_ADDR_PS_APPROACH_STATE (PAJ7620_ADDR_BASE + 0x6B) // R
#define PAJ7620_ADDR_PS_RAW_DATA (PAJ7620_ADDR_BASE + 0x6C)       // R

// REGISTER BANK 1
#define PAJ7620_ADDR_PS_GAIN (PAJ7620_ADDR_BASE + 0x44)          // RW
#define PAJ7620_ADDR_IDLE_S1_STEP_0 (PAJ7620_ADDR_BASE + 0x67)   // RW
#define PAJ7620_ADDR_IDLE_S1_STEP_1 (PAJ7620_ADDR_BASE + 0x68)   // RW
#define PAJ7620_ADDR_IDLE_S2_STEP_0 (PAJ7620_ADDR_BASE + 0x69)   // RW
#define PAJ7620_ADDR_IDLE_S2_STEP_1 (PAJ7620_ADDR_BASE + 0x6A)   // RW
#define PAJ7620_ADDR_OP_TO_S1_STEP_0 (PAJ7620_ADDR_BASE + 0x6B)  // RW
#define PAJ7620_ADDR_OP_TO_S1_STEP_1 (PAJ7620_ADDR_BASE + 0x6C)  // RW
#define PAJ7620_ADDR_OP_TO_S2_STEP_0 (PAJ7620_ADDR_BASE + 0x6D)  // RW
#define PAJ7620_ADDR_OP_TO_S2_STEP_1 (PAJ7620_ADDR_BASE + 0x6E)  // RW
#define PAJ7620_ADDR_OPERATION_ENABLE (PAJ7620_ADDR_BASE + 0x72) // RW

// PAJ7620_REGITER_BANK_SEL
#define PAJ7620_BANK0 PAJ7620_VAL(0, 0)
#define PAJ7620_BANK1 PAJ7620_VAL(1, 0)

// PAJ7620_ADDR_SUSPEND_CMD
#define PAJ7620_I2C_WAKEUP PAJ7620_VAL(1, 0)
#define PAJ7620_I2C_SUSPEND PAJ7620_VAL(0, 0)

// PAJ7620_ADDR_OPERATION_ENABLE
#define PAJ7620_ENABLE PAJ7620_VAL(1, 0)
#define PAJ7620_DISABLE PAJ7620_VAL(0, 0)

typedef enum
{
    BANK0 = 0,
    BANK1,
} bank_e;

#define GES_RIGHT_FLAG PAJ7620_VAL(1, 0)
#define GES_LEFT_FLAG PAJ7620_VAL(1, 1)
#define GES_UP_FLAG PAJ7620_VAL(1, 2)
#define GES_DOWN_FLAG PAJ7620_VAL(1, 3)
#define GES_FORWARD_FLAG PAJ7620_VAL(1, 4)
#define GES_BACKWARD_FLAG PAJ7620_VAL(1, 5)
#define GES_CLOCKWISE_FLAG PAJ7620_VAL(1, 6)
#define GES_COUNT_CLOCKWISE_FLAG PAJ7620_VAL(1, 7)
#define GES_WAVE_FLAG PAJ7620_VAL(1, 0)

#define INIT_REG_ARRAY_SIZE (sizeof(initRegisterArray) / sizeof(initRegisterArray[0]))

/***************************************************************************/
#define I2C_MASTER_SCL_IO 9         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 8         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define PAJ7620_ID 0x73

#define I2C_MASTER_WRITE 0
#define I2C_MASTER_READ 1
#define I2C_delay 20
#define I2C_delay2 5

jee_device_t paj7620u2IICDev;

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static esp_err_t iic_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    jee_i2c_master_send(paj7620u2IICDev, PAJ7620_ID, JEE_I2C_STOP, &reg_addr, 1);
    jee_i2c_master_recv(paj7620u2IICDev, PAJ7620_ID, JEE_I2C_STOP, data, len);
    return 0;
#if 0
    struct jee_i2c_msg msgs;
    uint8_t *dataBuff = (uint8_t *)malloc(len + 1);
    dataBuff[0] = reg_addr;
    msgs.flags = JEE_I2C_RD;
    msgs.addr = PAJ7620_ID;
    msgs.buf = dataBuff;
    msgs.len = len;
    i2c_cmd_handle_t cmd;
    esp_err_t espRc;
    jee_i2c_transfer(paj7620u2IICDev,&msgs,1);
    
    memcpy(data, &msgs.buf[1], len);

    free(dataBuff);
    return 0;
#endif
    // return i2c_master_write_read_device(I2C_MASTER_NUM, PAJ7620_ID, &reg_addr, 1, data, len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static esp_err_t iic_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret = ESP_OK;
    uint8_t write_buf[2] = {reg_addr, data};
    jee_i2c_master_send(paj7620u2IICDev, PAJ7620_ID, JEE_I2C_STOP, &write_buf, sizeof(write_buf));
#if 0
    
    struct jee_i2c_msg         msgs;
    uint8_t *dataBuff=(uint8_t *)malloc(2);
    dataBuff[0]=reg_addr;
    dataBuff[1]=data;
    msgs.flags=JEE_I2C_WR;
    msgs.addr=PAJ7620_ID;
    msgs.buf=dataBuff;
    msgs.len=2;
    jee_i2c_transfer(paj7620u2IICDev,&msgs,1);
    
    free(dataBuff);
#endif
#if 0
    int ret = ESP_OK;
    uint8_t write_buf[2] = {reg_addr, data};
    //   ret= iic_action_write_one_byte(PAJ7620_ID,reg_addr,data);
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, PAJ7620_ID, write_buf, sizeof(write_buf), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
#endif
    return ret;
}

static esp_err_t i2c_master_init(void)
{

    // 查找设备
    paj7620u2IICDev = jee_device_find("i2c0hard");
    if (paj7620u2IICDev == JEE_NULL)
    {
        printf("can not find %s Model\n", "i2c0hard");
        return ESP_FAIL;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(paj7620u2IICDev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open %s device\n", "i2c0hard");
        return ESP_FAIL;
    }
  
   // jee_device_control(paj7620u2IICDev,JEE_SENSOR_CTRL_USER_CMD_START+1,NULL);

      return ESP_OK;
#if 0
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
#endif
    return ESP_OK;
}

/***************************************************************************/
unsigned char initRegisterArray[][2] = {
    // Initial Gesture
    {0xEF, 0x00},
    {0x32, 0x29},
    {0x33, 0x01},
    {0x34, 0x00},
    {0x35, 0x01},
    {0x36, 0x00},
    {0x37, 0x07},
    {0x38, 0x17},
    {0x39, 0x06},
    {0x3A, 0x12},
    {0x3F, 0x00},
    {0x40, 0x02},
    {0x41, 0xFF},
    {0x42, 0x01},
    {0x46, 0x2D},
    {0x47, 0x0F},
    {0x48, 0x3C},
    {0x49, 0x00},
    {0x4A, 0x1E},
    {0x4B, 0x00},
    {0x4C, 0x20},
    {0x4D, 0x00},
    {0x4E, 0x1A},
    {0x4F, 0x14},
    {0x50, 0x00},
    {0x51, 0x10},
    {0x52, 0x00},
    {0x5C, 0x02},
    {0x5D, 0x00},
    {0x5E, 0x10},
    {0x5F, 0x3F},
    {0x60, 0x27},
    {0x61, 0x28},
    {0x62, 0x00},
    {0x63, 0x03},
    {0x64, 0xF7},
    {0x65, 0x03},
    {0x66, 0xD9},
    {0x67, 0x03},
    {0x68, 0x01},
    {0x69, 0xC8},
    {0x6A, 0x40},
    {0x6D, 0x04},
    {0x6E, 0x00},
    {0x6F, 0x00},
    {0x70, 0x80},
    {0x71, 0x00},
    {0x72, 0x00},
    {0x73, 0x00},
    {0x74, 0xF0},
    {0x75, 0x00},
    {0x80, 0x42},
    {0x81, 0x44},
    {0x82, 0x04},
    {0x83, 0x20},
    {0x84, 0x20},
    {0x85, 0x00},
    {0x86, 0x10},
    {0x87, 0x00},
    {0x88, 0x05},
    {0x89, 0x18},
    {0x8A, 0x10},
    {0x8B, 0x01},
    {0x8C, 0x37},
    {0x8D, 0x00},
    {0x8E, 0xF0},
    {0x8F, 0x81},
    {0x90, 0x06},
    {0x91, 0x06},
    {0x92, 0x1E},
    {0x93, 0x0D},
    {0x94, 0x0A},
    {0x95, 0x0A},
    {0x96, 0x0C},
    {0x97, 0x05},
    {0x98, 0x0A},
    {0x99, 0x41},
    {0x9A, 0x14},
    {0x9B, 0x0A},
    {0x9C, 0x3F},
    {0x9D, 0x33},
    {0x9E, 0xAE},
    {0x9F, 0xF9},
    {0xA0, 0x48},
    {0xA1, 0x13},
    {0xA2, 0x10},
    {0xA3, 0x08},
    {0xA4, 0x30},
    {0xA5, 0x19},
    {0xA6, 0x10},
    {0xA7, 0x08},
    {0xA8, 0x24},
    {0xA9, 0x04},
    {0xAA, 0x1E},
    {0xAB, 0x1E},
    {0xCC, 0x19},
    {0xCD, 0x0B},
    {0xCE, 0x13},
    {0xCF, 0x64},
    {0xD0, 0x21},
    {0xD1, 0x0F},
    {0xD2, 0x88},
    {0xE0, 0x01},
    {0xE1, 0x04},
    {0xE2, 0x41},
    {0xE3, 0xD6},
    {0xE4, 0x00},
    {0xE5, 0x0C},
    {0xE6, 0x0A},
    {0xE7, 0x00},
    {0xE8, 0x00},
    {0xE9, 0x00},
    {0xEE, 0x07},
    {0xEF, 0x01},
    {0x00, 0x1E},
    {0x01, 0x1E},
    {0x02, 0x0F},
    {0x03, 0x10},
    {0x04, 0x02},
    {0x05, 0x00},
    {0x06, 0xB0},
    {0x07, 0x04},
    {0x08, 0x0D},
    {0x09, 0x0E},
    {0x0A, 0x9C},
    {0x0B, 0x04},
    {0x0C, 0x05},
    {0x0D, 0x0F},
    {0x0E, 0x02},
    {0x0F, 0x12},
    {0x10, 0x02},
    {0x11, 0x02},
    {0x12, 0x00},
    {0x13, 0x01},
    {0x14, 0x05},
    {0x15, 0x07},
    {0x16, 0x05},
    {0x17, 0x07},
    {0x18, 0x01},
    {0x19, 0x04},
    {0x1A, 0x05},
    {0x1B, 0x0C},
    {0x1C, 0x2A},
    {0x1D, 0x01},
    {0x1E, 0x00},
    {0x21, 0x00},
    {0x22, 0x00},
    {0x23, 0x00},
    {0x25, 0x01},
    {0x26, 0x00},
    {0x27, 0x39},
    {0x28, 0x7F},
    {0x29, 0x08},
    {0x30, 0x03},
    {0x31, 0x00},
    {0x32, 0x1A},
    {0x33, 0x1A},
    {0x34, 0x07},
    {0x35, 0x07},
    {0x36, 0x01},
    {0x37, 0xFF},
    {0x38, 0x36},
    {0x39, 0x07},
    {0x3A, 0x00},
    {0x3E, 0xFF},
    {0x3F, 0x00},
    {0x40, 0x77},
    {0x41, 0x40},
    {0x42, 0x00},
    {0x43, 0x30},
    {0x44, 0xA0},
    {0x45, 0x5C},
    {0x46, 0x00},
    {0x47, 0x00},
    {0x48, 0x58},
    {0x4A, 0x1E},
    {0x4B, 0x1E},
    {0x4C, 0x00},
    {0x4D, 0x00},
    {0x4E, 0xA0},
    {0x4F, 0x80},
    {0x50, 0x00},
    {0x51, 0x00},
    {0x52, 0x00},
    {0x53, 0x00},
    {0x54, 0x00},
    {0x57, 0x80},
    {0x59, 0x10},
    {0x5A, 0x08},
    {0x5B, 0x94},
    {0x5C, 0xE8},
    {0x5D, 0x08},
    {0x5E, 0x3D},
    {0x5F, 0x99},
    {0x60, 0x45},
    {0x61, 0x40},
    {0x63, 0x2D},
    {0x64, 0x02},
    {0x65, 0x96},
    {0x66, 0x00},
    {0x67, 0x97},
    {0x68, 0x01},
    {0x69, 0xCD},
    {0x6A, 0x01},
    {0x6B, 0xB0},
    {0x6C, 0x04},
    {0x6D, 0x2C},
    {0x6E, 0x01},
    {0x6F, 0x32},
    {0x71, 0x00},
    {0x72, 0x01},
    {0x73, 0x35},
    {0x74, 0x00},
    {0x75, 0x33},
    {0x76, 0x31},
    {0x77, 0x01},
    {0x7C, 0x84},
    {0x7D, 0x03},
    {0x7E, 0x01},
};

/****************************************************************
 * Function Name: paj7620WriteReg
 * Description:  PAJ7620 Write reg cmd
 * Parameters: addr:reg address; cmd:function data
 * Return: error code; success: return 0
 ****************************************************************/
uint8_t paj7620WriteReg(uint8_t addr, uint8_t cmd)
{
    return iic_register_write_byte(addr, cmd);
}

/****************************************************************
 * Function Name: paj7620ReadReg
 * Description:  PAJ7620 read reg data
 * Parameters: addr:reg address;
 *			   qty:number of data to read, addr continuously increase;
 *			   data[]:storage memory start address
 * Return: error code; success: return 0
 ****************************************************************/
uint8_t paj7620ReadReg(uint8_t addr, uint8_t qty, uint8_t *data)
{
    return iic_register_read(addr, data, qty);
}

/****************************************************************
 * Function Name: paj7620SelectBank
 * Description:  PAJ7620 select register bank
 * Parameters: BANK0, BANK1
 * Return: none
 ****************************************************************/
void paj7620SelectBank(bank_e bank)
{
    switch (bank)
    {
    case BANK0:
        paj7620WriteReg(PAJ7620_REGITER_BANK_SEL, PAJ7620_BANK0);
        break;
    case BANK1:
        paj7620WriteReg(PAJ7620_REGITER_BANK_SEL, PAJ7620_BANK1);
        break;
    default:
        break;
    }
}

/****************************************************************
 * Function Name: paj7620Init
 * Description:  PAJ7620 REG INIT
 * Parameters: none
 * Return: error code; success: return 0
 ****************************************************************/
uint8_t paj7620Init(void)
{
    // Near_normal_mode_V5_6.15mm_121017 for 940nm
    int i = 0;
    uint8_t error;
    uint8_t data0 = 0, data1 = 0;
    // wakeup the sensor
    vTaskDelay(pdMS_TO_TICKS(700)); // Wait 700us for PAJ7620U2 to stabilize

    paj7620SelectBank(BANK0);

    error = paj7620ReadReg(0, 1, &data0);
    if (error != 0)
    {
        printf("paj7620ReadReg1 err %d,%02x\n", error, data0);
        // return error;
    }
    error = paj7620ReadReg(1, 1, &data1);
    if (error != 0)
    {
        printf("paj7620ReadReg1 err %d,%02x\n", error, data1);
        // return error;
    }
    printf("Addr0 =%02x", data0);

    printf(",  Addr1 =%02x", data1);

    if ((data0 != 0x20) || (data1 != 0x76))
    {
        printf("(data0 != 0x20 ) || (data1 != 0x76)\n");
        error = 0xff;
        // return 0xff;
    }
    if (data0 == 0x20)
    {

        printf("wake-up finish.\n");
    }

    for (i = 0; i < INIT_REG_ARRAY_SIZE; i++)
    {
        paj7620WriteReg(initRegisterArray[i][0], initRegisterArray[i][1]);
    }

    paj7620SelectBank(BANK0); // gesture flage reg in Bank0

    printf("Paj7620 initialize register finished.");
    return error;
}

/***************************************************************************/

#define GES_REACTION_TIME 500 // 500		// You can adjust the reaction time according to the actual circumstance.
#define GES_ENTRY_TIME 800    // 800		// When you want to recognize the Forward/Backward gestures, your gestures' reaction time must less than GES_ENTRY_TIME(0.8s).
#define GES_QUIT_TIME 1000    // 1000

void delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}
void setup()
{
    uint8_t error = 0;

    error = paj7620Init(); // initialize Paj7620 registers
    if (error)
    {
        printf("INIT ERROR,CODE:%02x\n", error);
    }
    else
    {
        printf("INIT OK\n");
    }
    printf("Please input your gestures:\n");
}
uint8_t genture_state = 0;
uint8_t genture_state_upload = 0;
void loop()
{
    uint8_t data = 0, data1 = 0, error;

    error = paj7620ReadReg(0x43, 1, &data); // Read Bank_0_Reg_0x43/0x44 for gesture result.
                                            //  printf("gesture %02x\n",data);
    if (!error)
    {
        switch (data) // When different gestures be detected, the variable 'data' will be set to different values by paj7620ReadReg(0x43, 1, &data).
        {
        case GES_RIGHT_FLAG:
            delay(GES_ENTRY_TIME);
            paj7620ReadReg(0x43, 1, &data);
            if (data == GES_FORWARD_FLAG)
            {
                printf("Forward\n");
                genture_state = 5;
                genture_state_upload = genture_state;
                delay(GES_QUIT_TIME);
            }
            else if (data == GES_BACKWARD_FLAG)
            {
                printf("Backward\n");
                genture_state = 6;
                genture_state_upload = genture_state;
                delay(GES_QUIT_TIME);
            }
            else
            {
                printf("Right\n");
                genture_state = 4;
                genture_state_upload = genture_state;
            }
            break;
        case GES_LEFT_FLAG:
            delay(GES_ENTRY_TIME);
            paj7620ReadReg(0x43, 1, &data);
            if (data == GES_FORWARD_FLAG)
            {
                printf("Forward\n");
                genture_state = 5;
                genture_state_upload = genture_state;

                delay(GES_QUIT_TIME);
            }
            else if (data == GES_BACKWARD_FLAG)
            {
                printf("Backward\n");
                genture_state = 6;
                genture_state_upload = genture_state;

                delay(GES_QUIT_TIME);
            }
            else
            {
                printf("Left\n");
                genture_state = 3;
                genture_state_upload = genture_state;
            }
            break;
        case GES_UP_FLAG:
            delay(GES_ENTRY_TIME);
            paj7620ReadReg(0x43, 1, &data);
            if (data == GES_FORWARD_FLAG)
            {
                printf("Forward\n");
                genture_state = 5;
                genture_state_upload = genture_state;

                delay(GES_QUIT_TIME);
            }
            else if (data == GES_BACKWARD_FLAG)
            {
                printf("Backward\n");
                genture_state = 6;
                genture_state_upload = genture_state;

                delay(GES_QUIT_TIME);
            }
            else
            {
                printf("Up\n");
                genture_state = 1;
                genture_state_upload = genture_state;
            }
            break;
        case GES_DOWN_FLAG:
            delay(GES_ENTRY_TIME);
            paj7620ReadReg(0x43, 1, &data);
            if (data == GES_FORWARD_FLAG)
            {
                printf("Forward\n");
                genture_state = 5;
                genture_state_upload = genture_state;

                delay(GES_QUIT_TIME);
            }
            else if (data == GES_BACKWARD_FLAG)
            {
                printf("Backward\n");
                genture_state = 6;
                genture_state_upload = genture_state;

                delay(GES_QUIT_TIME);
            }
            else
            {
                printf("Down\n");
                genture_state = 2;
                genture_state_upload = genture_state;
            }
            break;
        case GES_FORWARD_FLAG:
            printf("Forward\n");
            genture_state = 5;
            genture_state_upload = genture_state;

            delay(GES_QUIT_TIME);
            break;
        case GES_BACKWARD_FLAG:
            printf("Backward");
            genture_state = 6;
            genture_state_upload = genture_state;

            delay(GES_QUIT_TIME);
            break;
        case GES_CLOCKWISE_FLAG:
            printf("Clockwise\n");
            genture_state = 7;
            genture_state_upload = genture_state;

            break;
        case GES_COUNT_CLOCKWISE_FLAG:
            printf("anti-clockwise\n");
            genture_state = 8;
            genture_state_upload = genture_state;

            break;
        default:
            paj7620ReadReg(0x44, 1, &data1);
            if (data1 == GES_WAVE_FLAG)
            {
                genture_state = 9;
                genture_state_upload = genture_state;
                printf("wave\n");
            }
            break;
        }
    }
    delay(100);
}
/***************************************************************************/
static void action_task_example(void *arg)
{

    setup();

    while (1)
    {

        loop();
    }
}
void action_init(void)
{

    i2c_master_init();

    xTaskCreate(action_task_example, "action_task_example", 2048, NULL, 10, NULL);
}
void gesture_sensor_init(void)
{
    action_init();
}

uint8_t get_getsture_status(void)
{
    uint8_t data = genture_state;
    genture_state =0;
    return data;
}

