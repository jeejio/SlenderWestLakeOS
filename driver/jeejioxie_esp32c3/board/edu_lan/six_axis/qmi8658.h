#ifndef __QMI8658_H__
#define __QMI8658_H__


#include "string.h"

#define M_PI (3.14159265358979323846f)
#define ONE_G (9.807f)
#define QFABS(x) (((x) < 0.0f) ? (-1.0f * (x)) : (x))

//#define QMI8658_SYNC_SAMPLE_MODE
//#define QMI8658_SOFT_SELFTEST
//#define QMI8658_USE_CALI

#define QMI8658_USE_FIFO
#define QMI8658_USE_AMD
#define QMI8658_USE_PEDOMETER

#define QMI8658_SLAVE_ADDR_L 0x6a
#define QMI8658_SLAVE_ADDR_H 0x6b

#define QMI8658_DISABLE_ALL (0x0)
#define QMI8658_ACC_ENABLE (0x1)
#define QMI8658_GYR_ENABLE (0x2)
#define QMI8658_ACCGYR_ENABLE (QMI8658_ACC_ENABLE | QMI8658_GYR_ENABLE)

#define QMI8658_STATUS1_CMD_DONE (0x01)
#define QMI8658_STATUS1_WAKEUP_EVENT (0x04)

#define QMI8658_CTRL8_DATAVALID_EN 0x40 // bit6:1 int1, 0 int2
#define QMI8658_CTRL8_PEDOMETER_EN 0x10
#define QMI8658_CTRL8_SIGMOTION_EN 0x08
#define QMI8658_CTRL8_NOMOTION_EN 0x04
#define QMI8658_CTRL8_ANYMOTION_EN 0x02
#define QMI8658_CTRL8_TAP_EN 0x01

#define QMI8658_INT1_ENABLE 0x08
#define QMI8658_INT2_ENABLE 0x10

#define qmi8658_log printf

enum Qmi8658Register
{
    Qmi8658Register_WhoAmI = 0,
    Qmi8658Register_Revision,
    Qmi8658Register_Ctrl1,
    Qmi8658Register_Ctrl2,
    Qmi8658Register_Ctrl3,
    Qmi8658Register_Ctrl4,
    Qmi8658Register_Ctrl5,
    Qmi8658Register_Ctrl6,
    Qmi8658Register_Ctrl7,
    Qmi8658Register_Ctrl8,
    Qmi8658Register_Ctrl9,
    Qmi8658Register_Cal1_L = 11,
    Qmi8658Register_Cal1_H,
    Qmi8658Register_Cal2_L,
    Qmi8658Register_Cal2_H,
    Qmi8658Register_Cal3_L,
    Qmi8658Register_Cal3_H,
    Qmi8658Register_Cal4_L,
    Qmi8658Register_Cal4_H,
    Qmi8658Register_FifoWmkTh = 19,
    Qmi8658Register_FifoCtrl = 20,
    Qmi8658Register_FifoCount = 21,
    Qmi8658Register_FifoStatus = 22,
    Qmi8658Register_FifoData = 23,
    Qmi8658Register_StatusI2CM = 44,
    Qmi8658Register_StatusInt = 45,
    Qmi8658Register_Status0,
    Qmi8658Register_Status1,
    Qmi8658Register_Timestamp_L = 48,
    Qmi8658Register_Timestamp_M,
    Qmi8658Register_Timestamp_H,
    Qmi8658Register_Tempearture_L = 51,
    Qmi8658Register_Tempearture_H,
    Qmi8658Register_Ax_L = 53,
    Qmi8658Register_Ax_H,
    Qmi8658Register_Ay_L,
    Qmi8658Register_Ay_H,
    Qmi8658Register_Az_L,
    Qmi8658Register_Az_H,
    Qmi8658Register_Gx_L = 59,
    Qmi8658Register_Gx_H,
    Qmi8658Register_Gy_L,
    Qmi8658Register_Gy_H,
    Qmi8658Register_Gz_L,
    Qmi8658Register_Gz_H,
    Qmi8658Register_Cod_Status = 70,
    Qmi8658Register_firmware_id = 73,
    Qmi8658Register_Reset_done = 77,
    Qmi8658Register_uuid = 81,
    Qmi8658Register_Dvx_L = 81,
    Qmi8658Register_Dvx_H,
    Qmi8658Register_Dvy_L,
    Qmi8658Register_Dvy_H,
    Qmi8658Register_Dvz_L,
    Qmi8658Register_Dvz_H,

    Qmi8658Register_Pedo_L = 90,
    Qmi8658Register_Pedo_M = 91,
    Qmi8658Register_Pedo_H = 92,

    Qmi8658Register_Reset = 96
};

enum qmi8658_Ois_Register
{
    qmi8658_OIS_Reg_Ctrl1 = 0x02,
    qmi8658_OIS_Reg_Ctrl2,
    qmi8658_OIS_Reg_Ctrl3,
    qmi8658_OIS_Reg_Ctrl5 = 0x06,
    qmi8658_OIS_Reg_Ctrl7 = 0x08,
    qmi8658_OIS_Reg_StatusInt = 0x2D,
    qmi8658_OIS_Reg_Status0 = 0x2E,
    qmi8658_OIS_Reg_Ax_L = 0x33,
    qmi8658_OIS_Reg_Ax_H,
    qmi8658_OIS_Reg_Ay_L,
    qmi8658_OIS_Reg_Ay_H,
    qmi8658_OIS_Reg_Az_L,
    qmi8658_OIS_Reg_Az_H,

    qmi8658_OIS_Reg_Gx_L = 0x3B,
    qmi8658_OIS_Reg_Gx_H,
    qmi8658_OIS_Reg_Gy_L,
    qmi8658_OIS_Reg_Gy_H,
    qmi8658_OIS_Reg_Gz_L,
    qmi8658_OIS_Reg_Gz_H,
};

enum qmi8658_Ctrl9Command
{
    qmi8658_Ctrl9_Cmd_NOP = 0X00,
    qmi8658_Ctrl9_Cmd_GyroBias = 0X01,
    qmi8658_Ctrl9_Cmd_Rqst_Sdi_Mod = 0X03,
    qmi8658_Ctrl9_Cmd_Rst_Fifo = 0X04,
    qmi8658_Ctrl9_Cmd_Req_Fifo = 0X05,
    qmi8658_Ctrl9_Cmd_I2CM_Write = 0X06,
    qmi8658_Ctrl9_Cmd_WoM_Setting = 0x08,
    qmi8658_Ctrl9_Cmd_AccelHostDeltaOffset = 0x09,
    qmi8658_Ctrl9_Cmd_GyroHostDeltaOffset = 0x0A,
    qmi8658_Ctrl9_Cmd_EnableExtReset = 0x0B,
    qmi8658_Ctrl9_Cmd_EnableTap = 0x0C,
    qmi8658_Ctrl9_Cmd_EnablePedometer = 0x0D,
    qmi8658_Ctrl9_Cmd_Motion = 0x0E,
    qmi8658_Ctrl9_Cmd_CopyUsid = 0x10,
    qmi8658_Ctrl9_Cmd_SetRpu = 0x11,
    qmi8658_Ctrl9_Cmd_On_Demand_Cali = 0xA2,
    qmi8658_Ctrl9_Cmd_Dbg_WoM_Data_Enable = 0xF8
};

enum qmi8658_LpfConfig
{
    Qmi8658Lpf_Disable,
    Qmi8658Lpf_Enable
};

enum qmi8658_HpfConfig
{
    Qmi8658Hpf_Disable,
    Qmi8658Hpf_Enable
};

enum qmi8658_StConfig
{
    Qmi8658St_Disable,
    Qmi8658St_Enable
};

enum qmi8658_LpfMode
{
    A_LSP_MODE_0 = 0x00 << 1,
    A_LSP_MODE_1 = 0x01 << 1,
    A_LSP_MODE_2 = 0x02 << 1,
    A_LSP_MODE_3 = 0x03 << 1,

    G_LSP_MODE_0 = 0x00 << 5,
    G_LSP_MODE_1 = 0x01 << 5,
    G_LSP_MODE_2 = 0x02 << 5,
    G_LSP_MODE_3 = 0x03 << 5
};

enum qmi8658_AccRange
{
    Qmi8658AccRange_2g = 0x00 << 4,
    Qmi8658AccRange_4g = 0x01 << 4,
    Qmi8658AccRange_8g = 0x02 << 4,
    Qmi8658AccRange_16g = 0x03 << 4
};

enum qmi8658_AccOdr
{
    Qmi8658AccOdr_8000Hz = 0x00,
    Qmi8658AccOdr_4000Hz = 0x01,
    Qmi8658AccOdr_2000Hz = 0x02,
    Qmi8658AccOdr_1000Hz = 0x03,
    Qmi8658AccOdr_500Hz = 0x04,
    Qmi8658AccOdr_250Hz = 0x05,
    Qmi8658AccOdr_125Hz = 0x06,
    Qmi8658AccOdr_62_5Hz = 0x07,
    Qmi8658AccOdr_31_25Hz = 0x08,
    Qmi8658AccOdr_LowPower_128Hz = 0x0c,
    Qmi8658AccOdr_LowPower_21Hz = 0x0d,
    Qmi8658AccOdr_LowPower_11Hz = 0x0e,
    Qmi8658AccOdr_LowPower_3Hz = 0x0f
};

enum qmi8658_GyrRange
{
    Qmi8658GyrRange_16dps = 0 << 4,
    Qmi8658GyrRange_32dps = 1 << 4,
    Qmi8658GyrRange_64dps = 2 << 4,
    Qmi8658GyrRange_128dps = 3 << 4,
    Qmi8658GyrRange_256dps = 4 << 4,
    Qmi8658GyrRange_512dps = 5 << 4,
    Qmi8658GyrRange_1024dps = 6 << 4,
    Qmi8658GyrRange_2048dps = 7 << 4
};

/*!
 * \brief Gyroscope output rate configuration.
 */
enum qmi8658_GyrOdr
{
    Qmi8658GyrOdr_8000Hz = 0x00,
    Qmi8658GyrOdr_4000Hz = 0x01,
    Qmi8658GyrOdr_2000Hz = 0x02,
    Qmi8658GyrOdr_1000Hz = 0x03,
    Qmi8658GyrOdr_500Hz = 0x04,
    Qmi8658GyrOdr_250Hz = 0x05,
    Qmi8658GyrOdr_125Hz = 0x06,
    Qmi8658GyrOdr_62_5Hz = 0x07,
    Qmi8658GyrOdr_31_25Hz = 0x08
};

enum qmi8658_AccUnit
{
    Qmi8658AccUnit_g,
    Qmi8658AccUnit_ms2
};

enum qmi8658_GyrUnit
{
    Qmi8658GyrUnit_dps,
    Qmi8658GyrUnit_rads
};

enum qmi8658_FifoMode
{
    qmi8658_Fifo_Bypass = 0,
    qmi8658_Fifo_Fifo = 1,
    qmi8658_Fifo_Stream = 2,
    qmi8658_Fifo_StreamToFifo = 3
};

enum qmi8658_FifoWmkLevel
{
    qmi8658_Fifo_WmkEmpty = (0 << 4),
    qmi8658_Fifo_WmkOneQuarter = (1 << 4),
    qmi8658_Fifo_WmkHalf = (2 << 4),
    qmi8658_Fifo_WmkThreeQuarters = (3 << 4)
};

enum qmi8658_FifoSize
{
    qmi8658_Fifo_16 = (0 << 2),
    qmi8658_Fifo_32 = (1 << 2),
    qmi8658_Fifo_64 = (2 << 2),
    qmi8658_Fifo_128 = (3 << 2)
};

enum qmi8658_Interrupt
{
    qmi8658_Int_none,
    qmi8658_Int1,
    qmi8658_Int2,

    qmi8658_Int_total
};

enum qmi8658_InterruptState
{
    Qmi8658State_high = (1 << 7),
    Qmi8658State_low = (0 << 7)
};

#define QMI8658_CALI_DATA_NUM 200

typedef struct qmi8658_cali
{
    float acc[3];
    float gyr[3];
    float acc_bias[3];
    float gyr_bias[3];
    float acc_sum[3];
    float gyr_sum[3];
    unsigned char acc_cali_flag;
    unsigned char gyr_cali_flag;
    unsigned short acc_cali_num;
    unsigned short gyr_cali_num;
    //    unsigned char	acc_avg_num;
    //    unsigned char	gyr_avg_num;
} qmi8658_cali;

typedef struct
{
    unsigned char enSensors;
    enum qmi8658_AccRange accRange;
    enum qmi8658_AccOdr accOdr;
    enum qmi8658_GyrRange gyrRange;
    enum qmi8658_GyrOdr gyrOdr;
    unsigned char ctrl8_value;
#if defined(QMI8658_USE_FIFO)
    unsigned char fifo_ctrl;
#endif
} qmi8658_config;

typedef struct
{
    unsigned char slave;
    qmi8658_config cfg;
    unsigned short ssvt_a;
    unsigned short ssvt_g;
    unsigned int timestamp;
    unsigned int step;
    float imu[6];
} qmi8658_state;

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} quater_param_t;

typedef struct
{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} icm_param_t;

typedef struct
{
    float pitch;
    float roll;
    float yaw;
} euler_param_t;

typedef struct
{

    uint16_t Xdata;
    uint16_t Ydata;
    uint16_t Zdata;
} gyro_param_t;
/*
extern unsigned char qmi8658_write_reg(unsigned char reg, unsigned char value);
extern unsigned char qmi8658_read_reg(unsigned char reg, unsigned char* buf, unsigned short len);
extern unsigned char qmi8658_init(void);
extern void qmi8658_config_reg(unsigned char low_power);
extern void qmi8658_enableSensors(unsigned char enableFlags);
extern unsigned char qmi8658_readStatusInt(void);
extern unsigned char qmi8658_readStatus0(void);
extern unsigned char qmi8658_readStatus1(void);
extern float qmi8658_readTemp(void);
extern void qmi8658_read_timestamp(unsigned int *tim_count);
extern void qmi8658_read_xyz(float acc[3], float gyro[3],float eulerAngles[3]);

#if defined(QMI8658_USE_PEDOMETER)
extern unsigned int qmi8658_read_pedometer(void);
#endif
#if defined(QMI8658_USE_AMD)
void qmi8658_config_amd(void);
void qmi8658_enable_amd(unsigned char enable, enum qmi8658_Interrupt int_map, unsigned char low_power);
#endif
#if defined(QMI8658_USE_FIFO)
extern void qmi8658_config_fifo(unsigned char watermark,enum qmi8658_FifoSize size,enum qmi8658_FifoMode mode);
extern unsigned short qmi8658_read_fifo(unsigned char* data);
#endif
extern void qmi8658_send_ctl9cmd(enum qmi8658_Ctrl9Command cmd);

*/
typedef struct
{
    float yawAngle;               /* */
    float pithAngle;
    float rollAngle;
    float accelerationX;
    float accelerationY;
    float accelerationZ;
    float angularVelocityX;
    float angularVelocityY;
    float angularVelocityZ;

} hal_imu_onReceive_t;


  /**
   * @brief       姿态传感器初始化 
   * 
   *   NOTE:      IIC初始化，配置传感器参数等
   * 
   */
void imu_init(void);

  /**
   * @brief       获取偏航角（yaw）.
   *
   *   NOTE:      用四元素计算偏航角，偏航角是表示绕y轴旋转的角度
   *
   * @return      yaw-偏航角值   
   */
float imu_get_yaw_angle(void);

  /**
   * @brief       获取俯仰角（pitch）.
   *
   *   NOTE:      用四元素计算俯仰角，俯仰角是表示绕x轴旋转的角度
   *
   * @return      pitch-俯仰角值   
   */
float imu_get_pitch_angle(void);

  /**
   * @brief       获取翻滚脚（roll）.
   *
   *   NOTE:      用四元素计算翻滚脚，滚转脚是表示绕z轴旋转的角度
   *
   * @return      roll-翻滚脚值   
   */
float imu_get_roll_angle(void);

  /**
   * @brief       获取加速度X轴.
   *
   *   NOTE:      从传感器中直接读取加速度X轴的高低字节，得到raw_acc_xyz[0]，
   *              公式：
   *              ONE_G = 9.807f  
   *             {  
   *              加速度参数2g       g_imu.ssvt_a =  (1 << 14);
   *              加速度参数4g       g_imu.ssvt_a =  (1 << 13);
   *              加速度参数8g       g_imu.ssvt_a =  (1 << 12);
   *              加速度参数16g      g_imu.ssvt_a =  (1 << 11);
   *              }
   *              acc[0] = (float)(raw_acc_xyz[0] * ONE_G) / g_imu.ssvt_a;
   *
   * @return      X轴值-acc[0]   
   */
float imu_get_acceleration_x(void);

  /**
   * @brief       获取加速度Y轴.
   *
   *   NOTE:      从传感器中直接读取加速度Y轴的高低字节，得到raw_acc_xyz[1]，
   *              公式：
   *              ONE_G = 9.807f  
   *             {  
   *              加速度参数2g       g_imu.ssvt_a =  (1 << 14);
   *              加速度参数4g       g_imu.ssvt_a =  (1 << 13);
   *              加速度参数8g       g_imu.ssvt_a =  (1 << 12);
   *              加速度参数16g      g_imu.ssvt_a =  (1 << 11);
   *              }
   *              acc[1] = (float)(raw_acc_xyz[1] * ONE_G) / g_imu.ssvt_a;
   *
   * @return      Y轴值-acc[1]   
   */
float imu_get_acceleration_y(void);

  /**
   * @brief       获取加速度Z轴.
   *
   *   NOTE:      从传感器中直接读取加速度Z轴的高低字节，得到raw_acc_xyz[2]，
   *              公式：
   *              ONE_G = 9.807f  
   *             {  
   *              加速度参数2g       g_imu.ssvt_a =  (1 << 14);
   *              加速度参数4g       g_imu.ssvt_a =  (1 << 13);
   *              加速度参数8g       g_imu.ssvt_a =  (1 << 12);
   *              加速度参数16g      g_imu.ssvt_a =  (1 << 11);
   *              }
   *              acc[2] = (float)(raw_acc_xyz[2] * ONE_G) / g_imu.ssvt_a;
   *
   * @return      Z轴值-acc[2]   
   */
float imu_get_acceleration_z(void);

  /**
   * @brief       获取陀螺仪X轴.
   *
   *   NOTE:      从传感器中直接读取陀螺仪X轴的高低字节，得到raw_gyro_xyz[0]，
   *              公式：
   *             {  
   *              陀螺仪参数16dps         g_imu.ssvt_g =  2048;
   *              陀螺仪参数32dps         g_imu.ssvt_g =  1024;
   *              陀螺仪参数64dps         g_imu.ssvt_g =  512;
   *              陀螺仪参数128dps        g_imu.ssvt_g =  256;
   *              陀螺仪参数256dps        g_imu.ssvt_g =  128;
   *              陀螺仪参数512dps        g_imu.ssvt_g =  64;
   *              陀螺仪参数1024dps       g_imu.ssvt_g =  32;
   *              陀螺仪参数2048dps       g_imu.ssvt_g =  16;
   *              }
   *              gyro[0] = (float)(raw_gyro_xyz[0] * 0.01745f) / g_imu.ssvt_g;
   *
   * @return      X轴值-gyro[0]   
   */

float imu_get_angular_velocity_x(void);

  /**
   * @brief       获取陀螺仪Y轴.
   *
   *   NOTE:      从传感器中直接读取陀螺仪Y轴的高低字节，得到raw_gyro_xyz[1]，
   *              公式：
   *             {  
   *              陀螺仪参数16dps         g_imu.ssvt_g =  2048;
   *              陀螺仪参数32dps         g_imu.ssvt_g =  1024;
   *              陀螺仪参数64dps         g_imu.ssvt_g =  512;
   *              陀螺仪参数128dps        g_imu.ssvt_g =  256;
   *              陀螺仪参数256dps        g_imu.ssvt_g =  128;
   *              陀螺仪参数512dps        g_imu.ssvt_g =  64;
   *              陀螺仪参数1024dps       g_imu.ssvt_g =  32;
   *              陀螺仪参数2048dps       g_imu.ssvt_g =  16;
   *              }
   *              gyro[1] = (float)(raw_gyro_xyz[1] * 0.01745f) / g_imu.ssvt_g;
   *
   * @return      Y轴值-gyro[1]   
   */
float imu_get_angular_velocity_y(void);

  /**
   * @brief       获取陀螺仪Z轴.
   *
   *   NOTE:      从传感器中直接读取陀螺仪Z轴的高低字节，得到raw_gyro_xyz[2]，
   *              公式：
   *             {  
   *              陀螺仪参数16dps         g_imu.ssvt_g =  2048;
   *              陀螺仪参数32dps         g_imu.ssvt_g =  1024;
   *              陀螺仪参数64dps         g_imu.ssvt_g =  512;
   *              陀螺仪参数128dps        g_imu.ssvt_g =  256;
   *              陀螺仪参数256dps        g_imu.ssvt_g =  128;
   *              陀螺仪参数512dps        g_imu.ssvt_g =  64;
   *              陀螺仪参数1024dps       g_imu.ssvt_g =  32;
   *              陀螺仪参数2048dps       g_imu.ssvt_g =  16;
   *              }
   *              gyro[2] = (float)(raw_gyro_xyz[2] * 0.01745f) / g_imu.ssvt_g;
   *
   * @return      Z轴值-gyro[2]   
   */
float imu_get_angular_velocity_z(void);

  /**
   * @brief       上报所有传感器数据.
   *
   *   NOTE:      数据包括：欧拉角，加速度，陀螺仪
   *
   * @return      指向传感器数据的指针 
   * 
   *   NOTE:
   *           typedef struct
   *             {
   *                 float yawAngle;           //偏航角度          
   *                 float pithAngle;          //俯仰角度
   *                 float rollAngle;          //翻滚角度
   *                 float accelerationX;      //X轴加速度
   *                 float accelerationY;      //Y轴加速度
   *                 float accelerationZ;      //Z轴加速度
   *                 float angularVelocityX;   //X轴角速度
   *                 float angularVelocityY;   //Y轴角速度
   *                 float angularVelocityZ;   //Z轴角速度
   *             } hal_imu_onReceive_t;
   * 
   */
hal_imu_onReceive_t *hal_imu_on_receive(void);

  /**
   * @brief       获取所有传感器数据.
   *
   *   NOTE:      数据包括：欧拉角，加速度，陀螺仪
   *
   * @param[out]  传感器数据
   * 
   *   NOTE:
   *          
   *            pdata[0] = hal_imu_receive_param.accelerationX;
   *            pdata[1] = hal_imu_receive_param.accelerationY;
   *            pdata[2] = hal_imu_receive_param.accelerationZ;
   *            pdata[3] = hal_imu_receive_param.angularVelocityX;
   *            pdata[4] = hal_imu_receive_param.angularVelocityY;
   *            pdata[5] = hal_imu_receive_param.angularVelocityZ;
   *            pdata[6] = hal_imu_receive_param.pithAngle;
   *            pdata[7] = hal_imu_receive_param.rollAngle;
   *            pdata[8] = hal_imu_receive_param.yawAngle;
   * 
   */

void vSixAsixGetAllData(float *pdata);

#endif
