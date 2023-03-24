#include "qmi8658.h"

#include <stdio.h>
#include "string.h"
#include "esp_log.h"
#include "hal_i2c.h"
#include "hal_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "math.h"
#include "string.h"

hal_imu_onReceive_t hal_imu_receive_param;

#define qmi8658_printf printf
#define LOG printf

/* */
#define delta_T 0.004f     // 采样周期4ms 即频率250HZ
gyro_param_t GyroOffset;   // 陀螺仪校准值
bool gyroOffesetFlags = 0; //
uint32_t gyroOffesetCount = 0;

float I_ex, I_ey, I_ez;               // 误差积分
quater_param_t Q_info = {1, 0, 0, 0}; // 四元数初始化
euler_param_t eulerAngle;             // 欧拉角
icm_param_t icm_data;                 // 采集的六轴数值
float icm_kp = 0.17;                  // 加速度计的收敛速率比例增益
float icm_ki = 0.004;                 // 陀螺仪收敛速率的积分增益

static qmi8658_state g_imu;
#if defined(QMI8658_USE_CALI)
static qmi8658_cali g_cali;
#endif
void gyroOffsetInit(void);
void icmGetValues(void);
void icmAHRSupdate(icm_param_t *icm);

#define Kp 10.0f
#define Ki 0.008f
/* #define pi 3.14159265f */
#define halfT 0.002127f /*half the sample period*/
/* 参与计算的加速度单位g 陀螺仪单位是弧度/s()【度*pi/180=弧度】*/
float roll, pitch, yaw;
float exInt, eyInt, ezInt;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; /** quaternion of sensor frame relative to auxiliary frame */

void get_euler_angles(float gx, float gy, float gz, float ax, float ay, float az);

extern esp_err_t iic_register_read(uint8_t reg_addr, uint8_t *data, size_t len);

extern esp_err_t iic_register_write_byte(uint8_t reg_addr, uint8_t data);

#define I2C_MASTER_SCL_IO 9         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 8         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000
#define Six_axis_ID 0x6a // 7b

#define PIN_NUM_INT 3

void qmi8658_read_xyz(float acc[3], float gyro[3], float eulerAngles[3]);

unsigned char qmi8658_init(void);

#define QMI8658_I2C_DEV_NAME "i2c0hard"
static jee_device_t qmi8658_i2c_dev;

static esp_err_t qmi8658_i2c_master_init(void)
{
    /*
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
    */
    // i2c_driver_install(0, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

    // 设备查找和打开

    qmi8658_i2c_dev = jee_device_find(QMI8658_I2C_DEV_NAME);
    if (qmi8658_i2c_dev == NULL)
    {
        printf("can not find qmi8658_i2c_dev: %s \n", QMI8658_I2C_DEV_NAME);
        return -1;
    }
    else
    {
        printf("find qmi8658_i2c_dev: %s \n", qmi8658_i2c_dev->name);
    }

    if (jee_device_open(qmi8658_i2c_dev, JEE_DEVICE_FLAG_RDWR) != JEE_EOK)
    {
        printf("open qmi8658_i2c_dev failed! \n");
        return -1;
    }
    else
    {
        printf("open qmi8658_i2c_dev ok \n");
    }

    return 0;
}
esp_err_t iic_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    // i2c_master_write_read_device(I2C_MASTER_NUM, Six_axis_ID, &reg_addr, 1, data, len, 1000);
    jee_i2c_master_send(qmi8658_i2c_dev, Six_axis_ID, JEE_I2C_STOP, &reg_addr, 1);
    jee_i2c_master_recv(qmi8658_i2c_dev, Six_axis_ID, JEE_I2C_STOP, data, len);
    return 0;
}
esp_err_t iic_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};
    jee_i2c_master_send(qmi8658_i2c_dev, Six_axis_ID, JEE_I2C_STOP, &write_buf, sizeof(write_buf));
    // i2c_master_write_to_device(I2C_MASTER_NUM, Six_axis_ID, write_buf, sizeof(write_buf), 1000);
    return 0;
}

#define HAL_IMU_INIT_FAIL 0
#define HAL_IMU_INIT_SUCCESS 1
#define HAL_IMU_INIT_WAIT 2
#define HAL_IMU_INIT_DEF 3 /* no init*/

int hal_imu_init_flags = HAL_IMU_INIT_DEF; // 0 : init fail  1:init success   2:wait init  3.def no init

void Six_axis(void *arg)
{
    hal_imu_init_flags = HAL_IMU_INIT_WAIT;
    hal_imu_init_flags = qmi8658_init();
    float hal_angle_xyz[3];
    float hal_acc_xyz[3];
    float hal_gyro_xyz[3];
    while (hal_imu_init_flags)
    {
        if (jee_pin_read(PIN_NUM_INT))
        {
            qmi8658_read_xyz(hal_acc_xyz, hal_gyro_xyz, hal_angle_xyz);
            {
                hal_imu_receive_param.accelerationX = hal_acc_xyz[0];
                hal_imu_receive_param.accelerationY = hal_acc_xyz[1];
                hal_imu_receive_param.accelerationZ = hal_acc_xyz[2];
                hal_imu_receive_param.angularVelocityX = hal_gyro_xyz[0];
                hal_imu_receive_param.angularVelocityY = hal_gyro_xyz[1];
                hal_imu_receive_param.angularVelocityZ = hal_gyro_xyz[2];
                hal_imu_receive_param.pithAngle = hal_angle_xyz[0];
                hal_imu_receive_param.rollAngle = hal_angle_xyz[1];
                hal_imu_receive_param.yawAngle = hal_angle_xyz[2];
            }
            //printf("ang: %d %d %d  acc:  %d,%d,%d  gyr: %d,%d,%d\n", (short)hal_angle_xyz[0], (short)hal_angle_xyz[1], (short)hal_angle_xyz[2], (short)hal_acc_xyz[0], (short)hal_acc_xyz[1], (short)hal_acc_xyz[2], (short)hal_gyro_xyz[0], (short)hal_gyro_xyz[1], (short)hal_gyro_xyz[2]);
            vTaskDelay(pdMS_TO_TICKS(4));
        }
    }
    vTaskDelete(NULL);
}

unsigned char qmi8658_write_reg(unsigned char reg, unsigned char value)
{

    return iic_register_write_byte(reg, value);
}

unsigned char qmi8658_write_regs(unsigned char reg, unsigned char *value, unsigned char len)
{
    return iic_register_read(reg, value, len);
}

unsigned char qmi8658_read_reg(unsigned char reg, unsigned char *buf, unsigned short len)
{
    return iic_register_read(reg, buf, len);
}

void qmi8658_delay(unsigned int ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void qmi8658_delay_us(unsigned int us)
{
}

void qmi8658_axis_convert(float data_a[3], float data_g[3], int layout)
{
    float raw[3], raw_g[3];

    raw[0] = data_a[0];
    raw[1] = data_a[1];
    // raw[2] = data[2];
    raw_g[0] = data_g[0];
    raw_g[1] = data_g[1];
    // raw_g[2] = data_g[2];

    if (layout >= 4 && layout <= 7)
    {
        data_a[2] = -data_a[2];
        data_g[2] = -data_g[2];
    }

    if (layout % 2)
    {
        data_a[0] = raw[1];
        data_a[1] = raw[0];

        data_g[0] = raw_g[1];
        data_g[1] = raw_g[0];
    }
    else
    {
        data_a[0] = raw[0];
        data_a[1] = raw[1];

        data_g[0] = raw_g[0];
        data_g[1] = raw_g[1];
    }

    if ((layout == 1) || (layout == 2) || (layout == 4) || (layout == 7))
    {
        data_a[0] = -data_a[0];
        data_g[0] = -data_g[0];
    }
    if ((layout == 2) || (layout == 3) || (layout == 6) || (layout == 7))
    {
        data_a[1] = -data_a[1];
        data_g[1] = -data_g[1];
    }
}

#if defined(QMI8658_USE_CALI)
void qmi8658_data_cali(unsigned char sensor, float data[3])
{
    float data_diff[3];

    if (sensor == 1)
    {
        data_diff[0] = QFABS((data[0] - g_cali.acc[0]));
        data_diff[1] = QFABS((data[1] - g_cali.acc[1]));
        data_diff[2] = QFABS((data[2] - g_cali.acc[2]));
        g_cali.acc[0] = data[0];
        g_cali.acc[1] = data[1];
        g_cali.acc[2] = data[2];

        if (((data_diff[0] + data_diff[1] + data_diff[2]) < 0.5f) && (data[2] < 10.8f) && (data[2] > 8.8f))
        {
            if (g_cali.acc_cali_num == 0)
            {
                g_cali.acc_sum[0] = 0.0f;
                g_cali.acc_sum[1] = 0.0f;
                g_cali.acc_sum[2] = 0.0f;
            }
            if (g_cali.acc_cali_num < QMI8658_CALI_DATA_NUM)
            {
                g_cali.acc_cali_num++;
                g_cali.acc_sum[0] += data[0];
                g_cali.acc_sum[1] += data[1];
                g_cali.acc_sum[2] += data[2];
                if (g_cali.acc_cali_num == QMI8658_CALI_DATA_NUM)
                {
                    g_cali.acc_sum[0] = g_cali.acc_sum[0] / QMI8658_CALI_DATA_NUM;
                    g_cali.acc_sum[1] = g_cali.acc_sum[1] / QMI8658_CALI_DATA_NUM;
                    g_cali.acc_sum[2] = g_cali.acc_sum[2] / QMI8658_CALI_DATA_NUM;

                    g_cali.acc_bias[0] = 0.0f - g_cali.acc_sum[0];
                    g_cali.acc_bias[1] = 0.0f - g_cali.acc_sum[1];
                    g_cali.acc_bias[2] = 9.807f - g_cali.acc_sum[2];
                    g_cali.acc_cali_flag = 1;
                }
            }
        }
        else
        {
            g_cali.acc_cali_num = 0;
            g_cali.acc_sum[0] = 0.0f;
            g_cali.acc_sum[1] = 0.0f;
            g_cali.acc_sum[2] = 0.0f;
        }

        if (g_cali.acc_cali_flag)
        {
            data[0] += g_cali.acc_bias[0];
            data[1] += g_cali.acc_bias[1];
            data[2] += g_cali.acc_bias[2];
        }
    }
    else if (sensor == 2)
    {
        data_diff[0] = QFABS((data[0] - g_cali.gyr[0]));
        data_diff[1] = QFABS((data[1] - g_cali.gyr[1]));
        data_diff[2] = QFABS((data[2] - g_cali.gyr[2]));
        g_cali.gyr[0] = data[0];
        g_cali.gyr[1] = data[1];
        g_cali.gyr[2] = data[2];

        if (((data_diff[0] + data_diff[1] + data_diff[2]) < 0.05f) && ((data[0] > -1.0f) && (data[0] < 1.0f)) && ((data[1] > -1.0f) && (data[1] < 1.0f)) && ((data[2] > -1.0f) && (data[2] < 1.0f)))
        {
            if (g_cali.gyr_cali_num == 0)
            {
                g_cali.gyr_sum[0] = 0.0f;
                g_cali.gyr_sum[1] = 0.0f;
                g_cali.gyr_sum[2] = 0.0f;
            }
            if (g_cali.gyr_cali_num < QMI8658_CALI_DATA_NUM)
            {
                g_cali.gyr_cali_num++;
                g_cali.gyr_sum[0] += data[0];
                g_cali.gyr_sum[1] += data[1];
                g_cali.gyr_sum[2] += data[2];
                if (g_cali.gyr_cali_num == QMI8658_CALI_DATA_NUM)
                {
                    g_cali.gyr_sum[0] = g_cali.gyr_sum[0] / QMI8658_CALI_DATA_NUM;
                    g_cali.gyr_sum[1] = g_cali.gyr_sum[1] / QMI8658_CALI_DATA_NUM;
                    g_cali.gyr_sum[2] = g_cali.gyr_sum[2] / QMI8658_CALI_DATA_NUM;

                    g_cali.gyr_bias[0] = 0.0f - g_cali.gyr_sum[0];
                    g_cali.gyr_bias[1] = 0.0f - g_cali.gyr_sum[1];
                    g_cali.gyr_bias[2] = 0.0f - g_cali.gyr_sum[2];
                    g_cali.gyr_cali_flag = 1;
                }
            }
        }
        else
        {
            g_cali.gyr_cali_num = 0;
            g_cali.gyr_sum[0] = 0.0f;
            g_cali.gyr_sum[1] = 0.0f;
            g_cali.gyr_sum[2] = 0.0f;
        }

        if (g_cali.gyr_cali_flag)
        {
            data[0] += g_cali.gyr_bias[0];
            data[1] += g_cali.gyr_bias[1];
            data[2] += g_cali.gyr_bias[2];
        }
    }
}
#endif

void qmi8658_config_acc(enum qmi8658_AccRange range, enum qmi8658_AccOdr odr, enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable)
{
    unsigned char ctl_dada;

    switch (range)
    {
    case Qmi8658AccRange_2g:
        g_imu.ssvt_a = (1 << 14);
        break;
    case Qmi8658AccRange_4g:
        g_imu.ssvt_a = (1 << 13);
        break;
    case Qmi8658AccRange_8g:
        g_imu.ssvt_a = (1 << 12);
        break;
    case Qmi8658AccRange_16g:
        g_imu.ssvt_a = (1 << 11);
        break;
    default:
        range = Qmi8658AccRange_8g;
        g_imu.ssvt_a = (1 << 12);
    }
    if (stEnable == Qmi8658St_Enable)
        ctl_dada = (unsigned char)range | (unsigned char)odr | 0x80;
    else
        ctl_dada = (unsigned char)range | (unsigned char)odr;

    qmi8658_write_reg(Qmi8658Register_Ctrl2, ctl_dada);
    // set LPF & HPF
    qmi8658_read_reg(Qmi8658Register_Ctrl5, &ctl_dada, 1);
    ctl_dada &= 0xf0;
    if (lpfEnable == Qmi8658Lpf_Enable)
    {
        ctl_dada |= A_LSP_MODE_3;
        ctl_dada |= 0x01;
    }
    else
    {
        ctl_dada &= ~0x01;
    }
    // ctl_dada = 0x00;
    qmi8658_write_reg(Qmi8658Register_Ctrl5, ctl_dada);
    // set LPF & HPF
}

void qmi8658_config_gyro(enum qmi8658_GyrRange range, enum qmi8658_GyrOdr odr, enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable)
{
    // Set the CTRL3 register to configure dynamic range and ODR
    unsigned char ctl_dada;

    // Store the scale factor for use when processing raw data
    switch (range)
    {
    case Qmi8658GyrRange_16dps:
        g_imu.ssvt_g = 2048;
        break;
    case Qmi8658GyrRange_32dps:
        g_imu.ssvt_g = 1024;
        break;
    case Qmi8658GyrRange_64dps:
        g_imu.ssvt_g = 512;
        break;
    case Qmi8658GyrRange_128dps:
        g_imu.ssvt_g = 256;
        break;
    case Qmi8658GyrRange_256dps:
        g_imu.ssvt_g = 128;
        break;
    case Qmi8658GyrRange_512dps:
        g_imu.ssvt_g = 64;
        break;
    case Qmi8658GyrRange_1024dps:
        g_imu.ssvt_g = 32;
        break;
    case Qmi8658GyrRange_2048dps:
        g_imu.ssvt_g = 16;
        break;
        //		case Qmi8658GyrRange_4096dps:
        //			g_imu.ssvt_g = 8;
        //			break;
    default:
        range = Qmi8658GyrRange_512dps;
        g_imu.ssvt_g = 64;
        break;
    }

    if (stEnable == Qmi8658St_Enable)
        ctl_dada = (unsigned char)range | (unsigned char)odr | 0x80;
    else
        ctl_dada = (unsigned char)range | (unsigned char)odr;
    qmi8658_write_reg(Qmi8658Register_Ctrl3, ctl_dada);

    // Conversion from degrees/s to rad/s if necessary
    // set LPF & HPF
    qmi8658_read_reg(Qmi8658Register_Ctrl5, &ctl_dada, 1);
    ctl_dada &= 0x0f;
    if (lpfEnable == Qmi8658Lpf_Enable)
    {
        ctl_dada |= G_LSP_MODE_3;
        ctl_dada |= 0x10;
    }
    else
    {
        ctl_dada &= ~0x10;
    }
    // ctl_dada = 0x00;
    qmi8658_write_reg(Qmi8658Register_Ctrl5, ctl_dada);
    // set LPF & HPF
}

void qmi8658_send_ctl9cmd(enum qmi8658_Ctrl9Command cmd)
{
    unsigned char status1 = 0x00;
    unsigned short count = 0;

    qmi8658_write_reg(Qmi8658Register_Ctrl9, (unsigned char)cmd); // write commond to ctrl9
#if 1                                                             // defined(QMI8658_NEW_FIRMWARE)
    unsigned char status_reg = Qmi8658Register_StatusInt;
    unsigned char cmd_done = 0x80;
    // unsigned char status_reg = Qmi8658Register_Status1;
    // unsigned char cmd_done = 0x01;

    qmi8658_read_reg(status_reg, &status1, 1);
    while (((status1 & cmd_done) != cmd_done) && (count++ < 100)) // read statusINT until bit7 is 1
    {
        qmi8658_delay(1);
        qmi8658_read_reg(status_reg, &status1, 1);
    }
    qmi8658_log("ctrl9 cmd done1 count=%d\n", count);

    qmi8658_write_reg(Qmi8658Register_Ctrl9, qmi8658_Ctrl9_Cmd_NOP); // write commond  0x00 to ctrl9
    count = 0;
    qmi8658_read_reg(status_reg, &status1, 1);
    while (((status1 & cmd_done) == cmd_done) && (count++ < 100)) // read statusINT until bit7 is 0
    {
        qmi8658_delay(1); // 1 ms
        qmi8658_read_reg(status_reg, &status1, 1);
    }
    qmi8658_log("ctrl9 cmd done2 count=%d\n", count);
#else
    while (((status1 & QMI8658_STATUS1_CMD_DONE) == 0) && (count++ < 100))
    {
        qmi8658_delay(1);
        qmi8658_read_reg(Qmi8658Register_Status1, &status1, sizeof(status1));
    }
#endif
}

unsigned char qmi8658_readStatusInt(void)
{
    unsigned char status_int;

    qmi8658_read_reg(Qmi8658Register_StatusInt, &status_int, 1);

    return status_int;
}

unsigned char qmi8658_readStatus0(void)
{
    unsigned char status0;

    qmi8658_read_reg(Qmi8658Register_Status0, &status0, 1);

    return status0;
}

unsigned char qmi8658_readStatus1(void)
{
    unsigned char status1;

    qmi8658_read_reg(Qmi8658Register_Status1, &status1, 1);

    return status1;
}

float qmi8658_readTemp(void)
{
    unsigned char buf[2];
    short temp = 0;
    float temp_f = 0;

    qmi8658_read_reg(Qmi8658Register_Tempearture_L, buf, 2);
    temp = ((short)buf[1] << 8) | buf[0];
    temp_f = (float)temp / 256.0f;

    return temp_f;
}

void qmi8658_read_timestamp(unsigned int *tim_count)
{
    unsigned char buf[3];
    unsigned int timestamp;

    if (tim_count)
    {
        qmi8658_read_reg(Qmi8658Register_Timestamp_L, buf, 3);
        timestamp = (unsigned int)(((unsigned int)buf[2] << 16) | ((unsigned int)buf[1] << 8) | buf[0]);
        if (timestamp > g_imu.timestamp)
            g_imu.timestamp = timestamp;
        else
            g_imu.timestamp = (timestamp + 0x1000000 - g_imu.timestamp);

        *tim_count = g_imu.timestamp;
    }
}

void qmi8658_read_sensor_data(float acc[3], float gyro[3])
{
    unsigned char buf_reg[12];
    short raw_acc_xyz[3];
    short raw_gyro_xyz[3];

    qmi8658_read_reg(Qmi8658Register_Ax_L, buf_reg, 12);
    raw_acc_xyz[0] = (short)((unsigned short)(buf_reg[1] << 8) | (buf_reg[0]));
    raw_acc_xyz[1] = (short)((unsigned short)(buf_reg[3] << 8) | (buf_reg[2]));
    raw_acc_xyz[2] = (short)((unsigned short)(buf_reg[5] << 8) | (buf_reg[4]));

    raw_gyro_xyz[0] = (short)((unsigned short)(buf_reg[7] << 8) | (buf_reg[6]));
    raw_gyro_xyz[1] = (short)((unsigned short)(buf_reg[9] << 8) | (buf_reg[8]));
    raw_gyro_xyz[2] = (short)((unsigned short)(buf_reg[11] << 8) | (buf_reg[10]));

#if defined(QMI8658_UINT_MG_DPS)
    // mg
    acc[0] = (float)(raw_acc_xyz[0] * 1000.0f) / g_imu.ssvt_a;
    acc[1] = (float)(raw_acc_xyz[1] * 1000.0f) / g_imu.ssvt_a;
    acc[2] = (float)(raw_acc_xyz[2] * 1000.0f) / g_imu.ssvt_a;
#else
    // m/s2
    acc[0] = (float)(raw_acc_xyz[0] * ONE_G) / g_imu.ssvt_a;
    acc[1] = (float)(raw_acc_xyz[1] * ONE_G) / g_imu.ssvt_a;
    acc[2] = (float)(raw_acc_xyz[2] * ONE_G) / g_imu.ssvt_a;
#endif

#if defined(QMI8658_UINT_MG_DPS)
    // dps
    gyro[0] = (float)(raw_gyro_xyz[0] * 1.0f) / g_imu.ssvt_g;
    gyro[1] = (float)(raw_gyro_xyz[1] * 1.0f) / g_imu.ssvt_g;
    gyro[2] = (float)(raw_gyro_xyz[2] * 1.0f) / g_imu.ssvt_g;
#else
    // rad/s
    gyro[0] = (float)(raw_gyro_xyz[0] * 0.01745f) / g_imu.ssvt_g; // *pi/180
    gyro[1] = (float)(raw_gyro_xyz[1] * 0.01745f) / g_imu.ssvt_g;
    gyro[2] = (float)(raw_gyro_xyz[2] * 0.01745f) / g_imu.ssvt_g;
#endif
}

void qmi8658_read_xyz(float acc[3], float gyro[3], float eulerAngles[3])
{
    unsigned char status;
    unsigned char data_ready = 0;

#if defined(QMI8658_SYNC_SAMPLE_MODE)
    qmi8658_read_reg(Qmi8658Register_StatusInt, &status, 1);
    if (status & 0x01)
    {
        data_ready = 1;
        qmi8658_delay_us(6); // delay 6us
    }
#else
    qmi8658_read_reg(Qmi8658Register_Status0, &status, 1);
    if (status & 0x03)
    {
        data_ready = 1;
    }
#endif
    if (data_ready)
    {
        qmi8658_read_sensor_data(acc, gyro);
        // gyroOffsetInit();
        get_euler_angles(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]);
        eulerAngles[0] = pitch;
        eulerAngles[1] = roll;
        eulerAngles[2] = yaw;
        qmi8658_axis_convert(acc, gyro, 1);
        //       printf("acc: %d,%d,%d gyr:%d,%d,%d ",(short)acc[0],(short)acc[1],(short)acc[2],(short)gyro[0],(short)gyro[1],(short)gyro[2]);

#if defined(QMI8658_USE_CALI)
        qmi8658_data_cali(2, gyro);
#endif
        g_imu.imu[0] = acc[0];
        g_imu.imu[1] = acc[1];
        g_imu.imu[2] = acc[2];
        g_imu.imu[3] = gyro[0];
        g_imu.imu[4] = gyro[1];
        g_imu.imu[5] = gyro[2];
    }
    else
    {
        acc[0] = g_imu.imu[0];
        acc[1] = g_imu.imu[1];
        acc[2] = g_imu.imu[2];
        gyro[0] = g_imu.imu[3];
        gyro[1] = g_imu.imu[4];
        gyro[2] = g_imu.imu[5];
        //        qmi8658_log("data ready fail!\n");
    }
    /*
        if(gyroOffesetFlags)
        {
        //icmGetValues();
         icm_data.acc_x = g_imu.imu[0];
        icm_data.acc_y = g_imu.imu[1];
        icm_data.acc_z = g_imu.imu[2];

        icm_data.gyro_x = g_imu.imu[3];
        icm_data.gyro_y = g_imu.imu[4];
        icm_data.gyro_z = g_imu.imu[5];

        icmAHRSupdate(&icm_data);
        }
    */
}

void qmi8658_enableSensors(unsigned char enableFlags)
{
#if defined(QMI8658_SYNC_SAMPLE_MODE)
    qmi8658_write_reg(Qmi8658Register_Ctrl7, enableFlags | 0x80);
#else
    qmi8658_write_reg(Qmi8658Register_Ctrl7, enableFlags & 0x0f);
#endif
    qmi8658_delay(1);
}

void qmi8658_dump_reg(void)
{
    unsigned char read_data[8];

    qmi8658_read_reg(Qmi8658Register_Ctrl1, read_data, 8);
    qmi8658_log("Ctrl1[0x%x]\nCtrl2[0x%x]\nCtrl3[0x%x]\nCtrl4[0x%x]\nCtrl5[0x%x]\nCtrl6[0x%x]\nCtrl7[0x%x]\nCtrl8[0x%x]\n",
                read_data[0], read_data[1], read_data[2], read_data[3], read_data[4], read_data[5], read_data[6], read_data[7]);
}

// void qmi8658_soft_reset(void)
//{
//	qmi8658_log("qmi8658_soft_reset \n");
//	qmi8658_write_reg(Qmi8658Register_Reset, 0xb0);
//	qmi8658_delay(2000);
//	qmi8658_write_reg(Qmi8658Register_Reset, 0x00);
//	qmi8658_delay(5);
// }

void qmi8658_on_demand_cali(void)
{
    qmi8658_log("qmi8658_on_demand_cali start\n");
    qmi8658_write_reg(Qmi8658Register_Ctrl7, 0x03);
    qmi8658_delay(500); // delay 500ms above
    qmi8658_write_reg(Qmi8658Register_Ctrl9, (unsigned char)qmi8658_Ctrl9_Cmd_On_Demand_Cali);
    qmi8658_delay(2000); // delay 2000ms above
    qmi8658_write_reg(Qmi8658Register_Ctrl7, 0x00);
    qmi8658_log("qmi8658_on_demand_cali done\n");
}

void qmi8658_config_reg(unsigned char low_power)
{
    qmi8658_enableSensors(QMI8658_DISABLE_ALL);
    if (low_power)
    {
        g_imu.cfg.enSensors = QMI8658_ACC_ENABLE;
        g_imu.cfg.accRange = Qmi8658AccRange_8g;
        g_imu.cfg.accOdr = Qmi8658AccOdr_LowPower_21Hz;
        g_imu.cfg.gyrRange = Qmi8658GyrRange_1024dps;
        g_imu.cfg.gyrOdr = Qmi8658GyrOdr_125Hz;
    }
    else
    {
        g_imu.cfg.enSensors = QMI8658_ACCGYR_ENABLE;
        g_imu.cfg.accRange = Qmi8658AccRange_8g;
        g_imu.cfg.accOdr = Qmi8658AccOdr_250Hz;
        g_imu.cfg.gyrRange = Qmi8658GyrRange_1024dps;
        g_imu.cfg.gyrOdr = Qmi8658GyrOdr_250Hz;
    }

    if (g_imu.cfg.enSensors & QMI8658_ACC_ENABLE)
    {
        qmi8658_config_acc(g_imu.cfg.accRange, g_imu.cfg.accOdr, Qmi8658Lpf_Disable, Qmi8658St_Disable);
    }
    if (g_imu.cfg.enSensors & QMI8658_GYR_ENABLE)
    {
        qmi8658_config_gyro(g_imu.cfg.gyrRange, g_imu.cfg.gyrOdr, Qmi8658Lpf_Disable, Qmi8658St_Disable);
    }
}

unsigned char qmi8658_get_id(void)
{
    unsigned char qmi8658_chip_id = 0x00;
    unsigned char qmi8658_revision_id = 0x00;
    unsigned char qmi8658_slave[2] = {QMI8658_SLAVE_ADDR_L, QMI8658_SLAVE_ADDR_H};
    int retry = 0;
    unsigned char iCount = 0;
    unsigned char firmware_id[3];

    while (iCount < 2)
    {
        g_imu.slave = qmi8658_slave[iCount];
        retry = 0;
        while ((qmi8658_chip_id != 0x05) && (retry++ < 5))
        {
            qmi8658_read_reg(Qmi8658Register_WhoAmI, &qmi8658_chip_id, 1);
            qmi8658_log("Qmi8658Register_WhoAmI = 0x%x\n", qmi8658_chip_id);
        }
        if (qmi8658_chip_id == 0x05)
        {
            g_imu.cfg.ctrl8_value = 0xc0;
            qmi8658_write_reg(Qmi8658Register_Ctrl1, 0x60 | QMI8658_INT2_ENABLE); // QMI8658_INT1_ENABLE, QMI8658_INT2_ENABLE
            qmi8658_write_reg(Qmi8658Register_Ctrl7, 0x00);
            qmi8658_write_reg(Qmi8658Register_Ctrl8, g_imu.cfg.ctrl8_value);
            qmi8658_read_reg(Qmi8658Register_Revision, &qmi8658_revision_id, 1);
            qmi8658_read_reg(0x49, firmware_id, 3);
            qmi8658_log("qmi8658_init slave=0x%x Revision=0x%x\n", g_imu.slave, qmi8658_revision_id);
            qmi8658_log("Firmware ID[0x%x 0x%x 0x%x]\n", firmware_id[2], firmware_id[1], firmware_id[0]);

            break;
        }
        iCount++;
    }

    return qmi8658_chip_id;
}

#if defined(QMI8658_USE_AMD)
void qmi8658_config_amd(void)
{
    g_imu.cfg.ctrl8_value &= (~QMI8658_CTRL8_ANYMOTION_EN);
    qmi8658_write_reg(Qmi8658Register_Ctrl8, g_imu.cfg.ctrl8_value);

    qmi8658_write_reg(Qmi8658Register_Cal1_L, 0x03); // any motion X threshold U 3.5 first three bit(uint 1g)  last five bit (uint 1/32 g)
    qmi8658_write_reg(Qmi8658Register_Cal1_H, 0x03); // any motion Y threshold U 3.5 first three bit(uint 1g)  last five bit (uint 1/32 g)
    qmi8658_write_reg(Qmi8658Register_Cal2_L, 0x03); // any motion Z threshold U 3.5 first three bit(uint 1g)  last five bit (uint 1/32 g)
    qmi8658_write_reg(Qmi8658Register_Cal2_H, 0x02); // no motion X threshold U 3.5 first three bit(uint 1g)  last five bit (uint 1/32 g)
    qmi8658_write_reg(Qmi8658Register_Cal3_L, 0x02);
    qmi8658_write_reg(Qmi8658Register_Cal3_H, 0x02);

    qmi8658_write_reg(Qmi8658Register_Cal4_L, 0xf7); // MOTION_MODE_CTRL
    qmi8658_write_reg(Qmi8658Register_Cal4_H, 0x01); // value 0x01

    qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_Motion);

    qmi8658_write_reg(Qmi8658Register_Cal1_L, 0x03); // AnyMotionWindow.
    qmi8658_write_reg(Qmi8658Register_Cal1_H, 0x01); // NoMotionWindow
    qmi8658_write_reg(Qmi8658Register_Cal2_L, 0x2c); // SigMotionWaitWindow[7:0]
    qmi8658_write_reg(Qmi8658Register_Cal2_H, 0x01); // SigMotionWaitWindow [15:8]
    qmi8658_write_reg(Qmi8658Register_Cal3_L, 0x64); // SigMotionConfirmWindow[7:0]
    qmi8658_write_reg(Qmi8658Register_Cal3_H, 0x00); // SigMotionConfirmWindow[15:8]
    // qmi8658_write_reg(Qmi8658Register_Cal4_L, 0xf7);
    qmi8658_write_reg(Qmi8658Register_Cal4_H, 0x02); // value 0x02

    qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_Motion);
}

void qmi8658_enable_amd(unsigned char enable, enum qmi8658_Interrupt int_map, unsigned char low_power)
{
    qmi8658_enableSensors(QMI8658_DISABLE_ALL);
    if (int_map == qmi8658_Int1)
    {
        g_imu.cfg.ctrl8_value &= (~QMI8658_CTRL8_ANYMOTION_EN);
        g_imu.cfg.ctrl8_value |= QMI8658_CTRL8_DATAVALID_EN;
    }
    else if (int_map == qmi8658_Int2)
    {
        g_imu.cfg.ctrl8_value &= (~QMI8658_CTRL8_ANYMOTION_EN);
        g_imu.cfg.ctrl8_value &= (~QMI8658_CTRL8_DATAVALID_EN);
    }
    qmi8658_write_reg(Qmi8658Register_Ctrl8, g_imu.cfg.ctrl8_value);
    qmi8658_delay(2);

    if (enable)
    {
        unsigned char ctrl1;

        qmi8658_config_reg(low_power);

        qmi8658_read_reg(Qmi8658Register_Ctrl1, &ctrl1, 1);
        if (int_map == qmi8658_Int1)
        {
            ctrl1 |= QMI8658_INT1_ENABLE;
            qmi8658_write_reg(Qmi8658Register_Ctrl1, ctrl1); // enable int for dev-E
        }
        else if (int_map == qmi8658_Int2)
        {
            ctrl1 |= QMI8658_INT2_ENABLE;
            qmi8658_write_reg(Qmi8658Register_Ctrl1, ctrl1); // enable int for dev-E
        }
        g_imu.cfg.ctrl8_value |= QMI8658_CTRL8_ANYMOTION_EN;
        qmi8658_write_reg(Qmi8658Register_Ctrl8, g_imu.cfg.ctrl8_value);

        qmi8658_delay(1);
        qmi8658_enableSensors(g_imu.cfg.enSensors);
    }
}
#endif

#if defined(QMI8658_USE_PEDOMETER)
void qmi8658_config_pedometer(unsigned short odr)
{
    float finalRate = (float)(200.0f / odr);                              // 14.285
    unsigned short ped_sample_cnt = (unsigned short)(0x0032 / finalRate); // 6;//(unsigned short)(0x0032 / finalRate) ;
    unsigned short ped_fix_peak2peak = 0x00AC;                            // 0x0006;//0x00CC;
    unsigned short ped_fix_peak = 0x00AC;                                 // 0x0006;//0x00CC;
    unsigned short ped_time_up = (unsigned short)(200 / finalRate);
    unsigned char ped_time_low = (unsigned char)(20 / finalRate);
    unsigned char ped_time_cnt_entry = 8;
    unsigned char ped_fix_precision = 0;
    unsigned char ped_sig_count = 1; // ¼Æ²½Æ÷¼Ó1

    qmi8658_write_reg(Qmi8658Register_Cal1_L, ped_sample_cnt & 0xFF);
    qmi8658_write_reg(Qmi8658Register_Cal1_H, (ped_sample_cnt >> 8) & 0xFF);
    qmi8658_write_reg(Qmi8658Register_Cal2_L, ped_fix_peak2peak & 0xFF);
    qmi8658_write_reg(Qmi8658Register_Cal2_H, (ped_fix_peak2peak >> 8) & 0xFF);
    qmi8658_write_reg(Qmi8658Register_Cal3_L, ped_fix_peak & 0xFF);
    qmi8658_write_reg(Qmi8658Register_Cal3_H, (ped_fix_peak >> 8) & 0xFF);
    qmi8658_write_reg(Qmi8658Register_Cal4_H, 0x01);
    qmi8658_write_reg(Qmi8658Register_Cal4_L, 0x02);
    qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_EnablePedometer);

    qmi8658_write_reg(Qmi8658Register_Cal1_L, ped_time_up & 0xFF);
    qmi8658_write_reg(Qmi8658Register_Cal1_H, (ped_time_up >> 8) & 0xFF);
    qmi8658_write_reg(Qmi8658Register_Cal2_L, ped_time_low);
    qmi8658_write_reg(Qmi8658Register_Cal2_H, ped_time_cnt_entry);
    qmi8658_write_reg(Qmi8658Register_Cal3_L, ped_fix_precision);
    qmi8658_write_reg(Qmi8658Register_Cal3_H, ped_sig_count);
    qmi8658_write_reg(Qmi8658Register_Cal4_H, 0x02);
    qmi8658_write_reg(Qmi8658Register_Cal4_L, 0x02);
    qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_EnablePedometer);
}

void qmi8658_enable_pedometer(unsigned char enable)
{
    if (enable)
    {
        g_imu.cfg.ctrl8_value |= QMI8658_CTRL8_PEDOMETER_EN;
    }
    else
    {
        g_imu.cfg.ctrl8_value &= (~QMI8658_CTRL8_PEDOMETER_EN);
    }
    qmi8658_write_reg(Qmi8658Register_Ctrl8, g_imu.cfg.ctrl8_value);
}

unsigned int qmi8658_read_pedometer(void)
{
    unsigned char buf[3];

    qmi8658_read_reg(Qmi8658Register_Pedo_L, buf, 3); // 0x5a
    g_imu.step = (unsigned int)((buf[2] << 16) | (buf[1] << 8) | (buf[0]));

    return g_imu.step;
}
#endif

#if defined(QMI8658_USE_FIFO)
void qmi8658_config_fifo(unsigned char watermark, enum qmi8658_FifoSize size, enum qmi8658_FifoMode mode)
{
    g_imu.cfg.fifo_ctrl = (unsigned char)(size | mode);

    qmi8658_write_reg(Qmi8658Register_FifoCtrl, g_imu.cfg.fifo_ctrl);
    qmi8658_write_reg(Qmi8658Register_FifoWmkTh, watermark);

    qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_Rst_Fifo);
}

unsigned short qmi8658_read_fifo(unsigned char *data)
{
    unsigned char fifo_status[2] = {0, 0};
    unsigned char fifo_sensors = 1;
    unsigned short fifo_bytes = 0;
    unsigned short fifo_level = 0;

    if ((g_imu.cfg.fifo_ctrl & 0x03) != qmi8658_Fifo_Bypass)
    {
        // qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_Req_Fifo);

        qmi8658_read_reg(Qmi8658Register_FifoCount, fifo_status, 2);
        fifo_bytes = (unsigned short)(((fifo_status[1] & 0x03) << 8) | fifo_status[0]);
        if ((g_imu.cfg.enSensors == QMI8658_ACC_ENABLE) || (g_imu.cfg.enSensors == QMI8658_GYR_ENABLE))
        {
            fifo_sensors = 1;
        }
        else if (g_imu.cfg.enSensors == QMI8658_ACCGYR_ENABLE)
        {
            fifo_sensors = 2;
        }
        fifo_level = fifo_bytes / (3 * fifo_sensors);
        fifo_bytes = fifo_level * (6 * fifo_sensors);
        qmi8658_log("read fifo level : %d\n", fifo_level);
        if (fifo_level > 0)
        {
            qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_Req_Fifo);
#if 0
			for(int i=0; i<fifo_level; i++)
			{
				qmi8658_read_reg(Qmi8658Register_FifoData, &data[i*fifo_sensors*6], fifo_sensors*6);
			}
#else
            qmi8658_read_reg(Qmi8658Register_FifoData, data, fifo_bytes);
#endif
        }
        qmi8658_write_reg(Qmi8658Register_FifoCtrl, g_imu.cfg.fifo_ctrl);
        // qmi8658_send_ctl9cmd(qmi8658_Ctrl9_Cmd_Rst_Fifo);
    }

    return fifo_level;
}
#endif

#if defined(QMI8658_SOFT_SELFTEST)
unsigned char qmi8658_do_selftest(void)
{
    float acc[3], gyr[3];
    unsigned char status;
    unsigned short st_count, retry, imu_fail_count;
    float norm_acc, norm_gyo;

    imu_fail_count = 0;
    st_count = 0;
    qmi8658_delay(50);
    while (st_count++ < 20)
    {
        qmi8658_delay(1);
        status = 0;
        retry = 0;
        while (!(status & 0x03) && (retry++ < 50))
        {
            qmi8658_read_reg(Qmi8658Register_Status0, &status, 1);
            qmi8658_delay(1);
        }
        if ((status & 0x03))
        {
            qmi8658_read_sensor_data(acc, gyr);
            norm_acc = acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2];
            norm_gyo = gyr[0] * gyr[0] + gyr[1] * gyr[1] + gyr[2] * gyr[2];
            qmi8658_log("qmi8658_do_selftest-%d %f	%f\n", st_count, norm_acc, norm_gyo);
            if ((norm_acc < 0.01f) || (norm_gyo < 0.000001f))
            {
                imu_fail_count++;
            }
        }
        else
        {
            imu_fail_count++;
        }
    }

    if (imu_fail_count > 15)
    {
        qmi8658_log("qmi8658_do_selftest-fail\n");
        return 0;
    }
    else
    {
        qmi8658_log("qmi8658_do_selftest-ok\n");
        return 1;
    }
}
#endif

void qmi8658_do_hw_selftest(int enSensor)
{
    unsigned char status_int = 0x00;
    unsigned int retry = 0;
    unsigned char reg[6];
    short raw[3];
    float st_out[3];

    if (enSensor & QMI8658_ACC_ENABLE)
    {
        qmi8658_enableSensors(QMI8658_DISABLE_ALL);
        qmi8658_write_reg(Qmi8658Register_Ctrl2, Qmi8658AccRange_8g | Qmi8658AccOdr_125Hz | 0x80);
        status_int = 0;
        retry = 0;
        while (!(status_int & 0x01))
        {
            qmi8658_read_reg(Qmi8658Register_StatusInt, &status_int, 1);
            qmi8658_delay(1);
            if (retry++ > 5000)
            {
                qmi8658_log("wati int high timeout\n");
                break;
            }
        }
        qmi8658_write_reg(Qmi8658Register_Ctrl2, Qmi8658AccRange_8g | Qmi8658AccOdr_125Hz);
        retry = 0;
        status_int = 0x01;
        while ((status_int & 0x01))
        {
            qmi8658_read_reg(Qmi8658Register_StatusInt, &status_int, 1);
            qmi8658_delay(1);
            if (retry++ > 5000)
            {
                qmi8658_log("wati int low timeout\n");
                break;
            }
        }
        qmi8658_read_reg(Qmi8658Register_Dvx_L, reg, 6);
        raw[0] = (short)((unsigned short)(reg[1] << 8) | (reg[0]));
        raw[1] = (short)((unsigned short)(reg[3] << 8) | (reg[2]));
        raw[2] = (short)((unsigned short)(reg[5] << 8) | (reg[4]));
        st_out[0] = (float)(raw[0] * 1000.0f / 2048); // mg
        st_out[1] = (float)(raw[1] * 1000.0f / 2048);
        st_out[2] = (float)(raw[2] * 1000.0f / 2048);
        if ((QFABS(st_out[0]) > 200) && (QFABS(st_out[1]) > 200) && (QFABS(st_out[2]) > 200))
        {
            qmi8658_log("acc-selftest out[%f	%f	%f] Pass!\n", st_out[0], st_out[1], st_out[2]);
        }
        else
        {
            qmi8658_log("acc-selftest out[%f	%f	%f] Fail!\n", st_out[0], st_out[1], st_out[2]);
        }
    }

    if (enSensor & QMI8658_GYR_ENABLE)
    {
        qmi8658_enableSensors(QMI8658_DISABLE_ALL);
        qmi8658_write_reg(Qmi8658Register_Ctrl3, Qmi8658GyrRange_1024dps | Qmi8658GyrOdr_250Hz | 0x80);
        status_int = 0;
        retry = 0;
        while (!(status_int & 0x01))
        {
            qmi8658_read_reg(Qmi8658Register_StatusInt, &status_int, 1);
            qmi8658_delay(1);
            if (retry++ > 5000)
            {
                qmi8658_log("wati int high timeout\n");
                break;
            }
        }
        qmi8658_write_reg(Qmi8658Register_Ctrl3, Qmi8658GyrRange_1024dps | Qmi8658GyrOdr_250Hz);
        retry = 0;
        status_int = 0x01;
        while ((status_int & 0x01))
        {
            qmi8658_read_reg(Qmi8658Register_StatusInt, &status_int, 1);
            qmi8658_delay(1);
            if (retry++ > 5000)
            {
                qmi8658_log("wati int low timeout\n");
                break;
            }
        }
        qmi8658_read_reg(Qmi8658Register_Dvx_L, reg, 6);
        raw[0] = (short)((unsigned short)(reg[1] << 8) | (reg[0]));
        raw[1] = (short)((unsigned short)(reg[3] << 8) | (reg[2]));
        raw[2] = (short)((unsigned short)(reg[5] << 8) | (reg[4]));
        st_out[0] = (float)(raw[0] / 16.0f); // dps
        st_out[1] = (float)(raw[1] / 16.0f);
        st_out[2] = (float)(raw[2] / 16.0f);
        if ((QFABS(st_out[0]) > 300) && (QFABS(st_out[1]) > 300) && (QFABS(st_out[2]) > 300))
        {
            qmi8658_log("gyr-selftest out[%f	%f	%f] Pass!\n", st_out[0], st_out[1], st_out[2]);
        }
        else
        {
            qmi8658_log("gyr-selftest out[%f	%f	%f] Fail!\n", st_out[0], st_out[1], st_out[2]);
        }
    }
}

unsigned char qmi8658_init(void)
{
    if (qmi8658_get_id() == 0x05)
    {
#if defined(QMI8658_USE_HW_SELFTEST)
//        qmi8658_do_hw_selftest(QMI8658_ACCGYR_ENABLE);
//        qmi8658_do_hw_selftest(QMI8658_GYR_ENABLE);
#endif
        qmi8658_on_demand_cali();
        qmi8658_config_reg(0);

        qmi8658_enableSensors(g_imu.cfg.enSensors);
        qmi8658_delay(400); // ÑÓ³Ù400ms¶ÁÊý¾Ý
        qmi8658_log("qmi8658_init success\n");
        GyroOffset.Xdata = 0;
        GyroOffset.Ydata = 0;
        GyroOffset.Zdata = 0;
        return 1;
    }
    else
    {
        qmi8658_log("qmi8658_init fail\n");
        return 0;
    }
}
#if 0
float myRsqrt(float num)
{
    float halfx = 0.5f * num;
    float y = num;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);

    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));

    return y;
}

/**
 * @brief 陀螺仪零漂初始化
 * 通过采集一定数据求均值计算陀螺仪零点偏移值。
 * 后续 陀螺仪读取的数据 - 零飘值，即可去除零点偏移量。
 */
void gyroOffsetInit(void)
{
    if (gyroOffesetFlags == 0)
    {
        gyroOffesetCount++;

        GyroOffset.Xdata += g_imu.imu[3];
        GyroOffset.Ydata += g_imu.imu[4];
        GyroOffset.Zdata += g_imu.imu[5];

        if(gyroOffesetCount==100)
        {
        gyroOffesetFlags=1;
        GyroOffset.Xdata /= 100;
        GyroOffset.Ydata /= 100;
        GyroOffset.Zdata /= 100;
        }
    }
}

/**
 * @brief 将采集的数值转化为实际物理值, 并对陀螺仪进行去零漂处理
 * 加速度计初始化配置 -> 测量范围: ±8g        对应灵敏度: 4096 LSB/g
 * 陀螺仪初始化配置   -> 测量范围: ±1024 dps  对应灵敏度: 32 LSB/dps   (degree per second)
 * @tips: gyro = (gyro_val / 32) °/s = ((gyro_val / 32) * PI / 180) rad/s
 */
void icmGetValues(void)
{  
    float alpha = 0.3;

    //一阶低通滤波，单位g
    icm_data.acc_x = (((float)g_imu.imu[0]) * alpha) / 4096 + icm_data.acc_x * (1 - alpha);
    icm_data.acc_y = (((float)g_imu.imu[1]) * alpha) / 4096 + icm_data.acc_y * (1 - alpha);
    icm_data.acc_z = (((float)g_imu.imu[2]) * alpha) / 4096 + icm_data.acc_z * (1 - alpha);

    //! 陀螺仪角速度必须转换为弧度制角速度: deg/s -> rad/s
    icm_data.gyro_x = ((float)g_imu.imu[3] - GyroOffset.Xdata) * M_PI / 180 / 32.0f;
    icm_data.gyro_y = ((float)g_imu.imu[4] - GyroOffset.Ydata) * M_PI / 180 / 32.0f;
    icm_data.gyro_z = ((float)g_imu.imu[5] - GyroOffset.Zdata) * M_PI / 180 / 32.0f;
}

/**
 * @brief 用互补滤波算法解算陀螺仪姿态(即利用加速度计修正陀螺仪的积分误差)
 * 加速度计对振动之类的噪声比较敏感，长期数据计算出的姿态可信；陀螺仪对振动噪声不敏感，短期数据可信，但长期使用积分误差严重(内部积分算法放大静态误差)。
 * 因此使用姿态互补滤波，短期相信陀螺仪，长期相信加速度计。
 * @tips: n - 导航坐标系； b - 载体坐标系
 */
void icmAHRSupdate(icm_param_t *icm)
{
    float halfT = 0.5 * delta_T; // 采样周期一半
    float vx, vy, vz;            // 当前姿态计算得来的重力在三轴上的分量
    float ex, ey, ez;            // 当前加速计测得的重力加速度在三轴上的分量与用当前姿态计算得来的重力在三轴上的分量的误差

    float q0 = Q_info.q0; //四元数
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;

    float q0q0 = q0 * q0; //先相乘，方便后续计算
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
//   qmi8658_log("updata: %d,%d,%d  %d,%d,%d\n",icm->acc_x,icm->acc_y,icm->acc_z,icm->gyro_x,icm->gyro_y,icm->gyro_z);

    // 正常静止状态为-g 反作用力。
    if (icm->acc_x * icm->acc_y * icm->acc_z == 0) // 加计处于自由落体状态时(此时g = 0)不进行姿态解算，因为会产生分母无穷大的情况
        return;

    // 对加速度数据进行归一化 得到单位加速度 (a^b -> 载体坐标系下的加速度)
    float norm = myRsqrt(icm->acc_x * icm->acc_x + icm->acc_y * icm->acc_y + icm->acc_z * icm->acc_z);
    qmi8658_log("norm %d",norm*1000);
    icm->acc_x = icm->acc_x * norm;
    icm->acc_y = icm->acc_y * norm;
    icm->acc_z = icm->acc_z * norm;

    // 载体坐标系下重力在三个轴上的分量
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    // g^b 与 a^b 做向量叉乘，得到陀螺仪的校正补偿向量e的系数
    ex = icm->acc_y * vz - icm->acc_z * vy;
    ey = icm->acc_z * vx - icm->acc_x * vz;
    ez = icm->acc_x * vy - icm->acc_y * vx;

    // 误差累加
    I_ex += halfT * ex;
    I_ey += halfT * ey;
    I_ez += halfT * ez;
    qmi8658_log("updata2: %d,%d,%d  %d,%d,%d\n",icm->acc_x,icm->acc_y,icm->acc_z,icm->gyro_x,icm->gyro_y,icm->gyro_z);

    // 使用PI控制器消除向量积误差(陀螺仪漂移误差)
    icm->gyro_x = icm->gyro_x + icm_kp * ex + icm_ki * I_ex;
    icm->gyro_y = icm->gyro_y + icm_kp * ey + icm_ki * I_ey;
    icm->gyro_z = icm->gyro_z + icm_kp * ez + icm_ki * I_ez;

    // 一阶龙格库塔法求解四元数微分方程，其中halfT为测量周期的1/2，gx gy gz为b系陀螺仪角速度。
    q0 = q0 + (-q1 * icm->gyro_x - q2 * icm->gyro_y - q3 * icm->gyro_z) * halfT;
    q1 = q1 + (q0 * icm->gyro_x + q2 * icm->gyro_z - q3 * icm->gyro_y) * halfT;
    q2 = q2 + (q0 * icm->gyro_y - q1 * icm->gyro_z + q3 * icm->gyro_x) * halfT;
    q3 = q3 + (q0 * icm->gyro_z + q1 * icm->gyro_y - q2 * icm->gyro_x) * halfT;

    // 单位化四元数在空间旋转时不会拉伸，仅有旋转角度，下面算法类似线性代数里的正交变换
    norm = myRsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm; // 用全局变量记录上一次计算的四元数值
    qmi8658_log("q0 :%d  %d   %d  %d" ,q0,q1,q2,q3);

    qmi8658_log("updata3: %d,%d,%d  %d,%d,%d\n",icm->acc_x,icm->acc_y,icm->acc_z,icm->gyro_x,icm->gyro_y,icm->gyro_z);


    eulerAngle.pitch = asin(2 * q0 * q2 - 2 * q1 * q3) * 180 / M_PI;
    eulerAngle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / M_PI;
    eulerAngle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / M_PI;
    qmi8658_log("pitch :%d,roll :%d,yaw :%d \n", eulerAngle.pitch,eulerAngle.roll,eulerAngle.yaw);

    
}

void CalulateEulerAngle(void)
{
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    // atan2返回输入坐标点与坐标原点连线与X轴正方形夹角的弧度值
    eulerAngle.pitch = asin(2 * q0 * q2 - 2 * q1 * q3) * 180 / M_PI;
    eulerAngle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / M_PI;
    eulerAngle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / M_PI;
}
#endif
/* 魔法函数InvSqrt()相当于1.0/sqrt() */
static float invSqrt(float number)
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = *((long *)&y);
    i = 0x5f375a86 - (i >> 1);
    y = *((float *)&i);
    y = y * (f - (x * y * y));
    return y;
}

void get_euler_angles(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez;

    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;

    float q1q1 = q1 * q1;
    float q1q3 = q1 * q3;

    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;

    float q3q3 = q3 * q3;
    //    printf("acc: %.2f,%.2f,%.2f gyr:%.2f,%.2f,%.2f ",ax,ay,az,gx,gy,gz);

    if (ax * ay * az == 0)
        return;
    /* 对加速度数据进行归一化处理 */
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax = ax * recipNorm;
    ay = ay * recipNorm;
    az = az * recipNorm;
    /* DCM矩阵旋转 */
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    /* 在机体坐标系下做向量叉积得到补偿数据 */
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;
    /* 对误差进行PI计算，补偿角速度 */
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;
    /* 按照四元素微分公式进行四元素更新 */
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    q0 = q0 * recipNorm;
    q1 = q1 * recipNorm;
    q2 = q2 * recipNorm;
    q3 = q3 * recipNorm;

    roll = atan2f(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3f;
    pitch = asinf(2 * q1 * q3 - 2 * q0 * q2) * 57.3f;
    yaw = -atan2f(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 57.3f;

    // printf("pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);
    //  printf("pitch:%d roll:%d yaw:%d\r\n",(short)pitch,(short)roll,(short)yaw);
}
#if 0
float hal_imu_get_yaw_angle(void);
float hal_imu_get_pitch_angle(void);
float hal_imu_get_roll_angle(void);
float hal_imu_get_acceleration_x(void);
float hal_imu_get_acceleration_y(void);
float hal_imu_get_acceleration_z(void);
float hal_imu_get_angular_velocity_x(void);
float hal_imu_get_angular_velocity_y(void);
float hal_imu_get_angular_velocity_z(void);
hal_imu_onReceive_t *hal_imu_on_receive(void);
void hal_imu_test(void *arg)
{
    uint8_t data=0;
    hal_imu_onReceive_t *test;
    vTaskDelay(pdMS_TO_TICKS(5000));

    while (1)
    {
        printf("data:");
        printf("%d ",(short)hal_imu_get_yaw_angle());
        printf("%d ",(short)hal_imu_get_pitch_angle());
        printf("%d ",(short)hal_imu_get_roll_angle());
        printf("%d ",(short)hal_imu_get_acceleration_x());
        printf("%d ",(short)hal_imu_get_acceleration_y());
        printf("%d ",(short)hal_imu_get_acceleration_z());
        printf("%d ",(short)hal_imu_get_angular_velocity_x());
        printf("%d ",(short)hal_imu_get_angular_velocity_y());
        printf("%d \n",(short)hal_imu_get_angular_velocity_z());
        test=hal_imu_on_receive();
        printf("all :%d %d %d %d %d %d %d %d %d \n",(short)test->yawAngle,(short)test->pithAngle,(short)test->rollAngle,
        (short)test->accelerationX,(short)test->angularVelocityY,(short)test->accelerationZ,(short)test->angularVelocityX,(short)test->angularVelocityY,(short)test->angularVelocityZ);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL);
}
#endif
void imu_init(void)
{
    qmi8658_i2c_master_init();
    xTaskCreate(Six_axis, "Six_axis", 4096, NULL, 10, NULL);
    //   xTaskCreate(hal_imu_test, "hal_imu_test", 4096, NULL, 10, NULL);
}

float imu_get_yaw_angle(void)
{
    switch (hal_imu_init_flags)
    {
    case HAL_IMU_INIT_FAIL:
        printf("HAL_IMU_INIT_FAIL\n");
        break;
    case HAL_IMU_INIT_SUCCESS:
        return hal_imu_receive_param.yawAngle;

        break;
    case HAL_IMU_INIT_WAIT:
        printf("HAL_IMU_INIT_WAIT\n");
        break;
    case HAL_IMU_INIT_DEF:
        printf("HAL_IMU_INIT_DEF\n");
        break;
    default:
        break;
    }
    return 0;
}

float imu_get_pitch_angle(void)
{
    switch (hal_imu_init_flags)
    {
    case HAL_IMU_INIT_FAIL:
        printf("HAL_IMU_INIT_FAIL\n");
        break;
    case HAL_IMU_INIT_SUCCESS:
        return hal_imu_receive_param.pithAngle;

        break;
    case HAL_IMU_INIT_WAIT:
        printf("HAL_IMU_INIT_WAIT\n");
        break;
    case HAL_IMU_INIT_DEF:
        printf("HAL_IMU_INIT_DEF\n");
        break;
    default:
        break;
    }
    return 0;
}

float imu_get_roll_angle(void)
{
    switch (hal_imu_init_flags)
    {
    case HAL_IMU_INIT_FAIL:
        printf("HAL_IMU_INIT_FAIL\n");
        break;
    case HAL_IMU_INIT_SUCCESS:
        return hal_imu_receive_param.rollAngle;

        break;
    case HAL_IMU_INIT_WAIT:
        printf("HAL_IMU_INIT_WAIT\n");
        break;
    case HAL_IMU_INIT_DEF:
        printf("HAL_IMU_INIT_DEF\n");
        break;
    default:
        break;
    }
    return 0;
}

float imu_get_acceleration_x(void)
{
    switch (hal_imu_init_flags)
    {
    case HAL_IMU_INIT_FAIL:
        printf("HAL_IMU_INIT_FAIL\n");
        break;
    case HAL_IMU_INIT_SUCCESS:
        return hal_imu_receive_param.accelerationX;
        break;
    case HAL_IMU_INIT_WAIT:
        printf("HAL_IMU_INIT_WAIT\n");
        break;
    case HAL_IMU_INIT_DEF:
        printf("HAL_IMU_INIT_DEF\n");
        break;
    default:
        break;
    }
    return 0;
}

float imu_get_acceleration_y(void)
{
    switch (hal_imu_init_flags)
    {
    case HAL_IMU_INIT_FAIL:
        printf("HAL_IMU_INIT_FAIL\n");
        break;
    case HAL_IMU_INIT_SUCCESS:
        return hal_imu_receive_param.accelerationY;
        break;
    case HAL_IMU_INIT_WAIT:
        printf("HAL_IMU_INIT_WAIT\n");
        break;
    case HAL_IMU_INIT_DEF:
        printf("HAL_IMU_INIT_DEF\n");
        break;
    default:
        break;
    }
    return 0;
}

float imu_get_acceleration_z(void)
{
    switch (hal_imu_init_flags)
    {
    case HAL_IMU_INIT_FAIL:
        printf("HAL_IMU_INIT_FAIL\n");
        break;
    case HAL_IMU_INIT_SUCCESS:
        return hal_imu_receive_param.accelerationZ;
        break;
    case HAL_IMU_INIT_WAIT:
        printf("HAL_IMU_INIT_WAIT\n");
        break;
    case HAL_IMU_INIT_DEF:
        printf("HAL_IMU_INIT_DEF\n");
        break;
    default:
        break;
    }
    return 0;
}

float imu_get_angular_velocity_x(void)
{
    switch (hal_imu_init_flags)
    {
    case HAL_IMU_INIT_FAIL:
        printf("HAL_IMU_INIT_FAIL\n");
        break;
    case HAL_IMU_INIT_SUCCESS:
        return hal_imu_receive_param.angularVelocityX;
        break;
    case HAL_IMU_INIT_WAIT:
        printf("HAL_IMU_INIT_WAIT\n");
        break;
    case HAL_IMU_INIT_DEF:
        printf("HAL_IMU_INIT_DEF\n");
        break;
    default:
        break;
    }
    return 0;
}

float imu_get_angular_velocity_y(void)
{
    switch (hal_imu_init_flags)
    {
    case HAL_IMU_INIT_FAIL:
        printf("HAL_IMU_INIT_FAIL\n");
        break;
    case HAL_IMU_INIT_SUCCESS:
        return hal_imu_receive_param.angularVelocityY;
        break;
    case HAL_IMU_INIT_WAIT:
        printf("HAL_IMU_INIT_WAIT\n");
        break;
    case HAL_IMU_INIT_DEF:
        printf("HAL_IMU_INIT_DEF\n");
        break;
    default:
        break;
    }
    return 0;
}

float imu_get_angular_velocity_z(void)
{
    switch (hal_imu_init_flags)
    {
    case HAL_IMU_INIT_FAIL:
        printf("HAL_IMU_INIT_FAIL\n");
        break;
    case HAL_IMU_INIT_SUCCESS:
        return hal_imu_receive_param.angularVelocityZ;
        break;
    case HAL_IMU_INIT_WAIT:
        printf("HAL_IMU_INIT_WAIT\n");
        break;
    case HAL_IMU_INIT_DEF:
        printf("HAL_IMU_INIT_DEF\n");
        break;
    default:
        break;
    }
    return 0;
}

hal_imu_onReceive_t *hal_imu_on_receive(void)
{
    switch (hal_imu_init_flags)
    {
    case HAL_IMU_INIT_FAIL:
        printf("HAL_IMU_INIT_FAIL\n");
        break;
    case HAL_IMU_INIT_SUCCESS:
        return &hal_imu_receive_param;
        break;
    case HAL_IMU_INIT_WAIT:
        printf("HAL_IMU_INIT_WAIT\n");
        break;
    case HAL_IMU_INIT_DEF:
        printf("HAL_IMU_INIT_DEF\n");
        break;
    default:
        break;
    }
    return NULL;
}

void vSixAsixGetAllData(float *pdata)
{
    switch (hal_imu_init_flags)
    {
    case HAL_IMU_INIT_FAIL:
        printf("HAL_IMU_INIT_FAIL\n");
        break;
    case HAL_IMU_INIT_SUCCESS:
    {
        pdata[0] = hal_imu_receive_param.accelerationX;
        pdata[1] = hal_imu_receive_param.accelerationY;
        pdata[2] = hal_imu_receive_param.accelerationZ;
        pdata[3] = hal_imu_receive_param.angularVelocityX;
        pdata[4] = hal_imu_receive_param.angularVelocityY;
        pdata[5] = hal_imu_receive_param.angularVelocityZ;
        pdata[6] = hal_imu_receive_param.pithAngle;
        pdata[7] = hal_imu_receive_param.rollAngle;
        pdata[8] = hal_imu_receive_param.yawAngle;
    }

    break;
    case HAL_IMU_INIT_WAIT:
        printf("HAL_IMU_INIT_WAIT\n");
        break;
    case HAL_IMU_INIT_DEF:
        printf("HAL_IMU_INIT_DEF\n");
        break;
    default:
        break;
    }
}
