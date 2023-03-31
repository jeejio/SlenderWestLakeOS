/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-30     zhengqian    first version
 */

#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "freertos/device.h"
#include "hal_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef JEE_USING_RTC
#define  jee_sensor_get_ts()  time(JEE_NULL)   /* API for the sensor to get the timestamp */
#else
#define  jee_sensor_get_ts()  jee_tick_get()   /* API for the sensor to get the timestamp */
#endif

#define  JEE_PIN_NONE                   0xFFFF    /* RT PIN NONE */
#define  JEE_DEVICE_FLAG_FIFO_RX        0x200     /* Flag to use when the sensor is open by fifo mode */

#define  JEE_SENSOR_MODULE_MAX          (3)       /* The maximum number of members of a sensor module */

/* Sensor types */

#define JEE_SENSOR_CLASS_NONE           (0)
#define JEE_SENSOR_CLASS_ACCE           (1)  /* Accelerometer     */
#define JEE_SENSOR_CLASS_GYRO           (2)  /* Gyroscope         */
#define JEE_SENSOR_CLASS_MAG            (3)  /* Magnetometer      */
#define JEE_SENSOR_CLASS_TEMP           (4)  /* Temperature       */
#define JEE_SENSOR_CLASS_HUMI           (5)  /* Relative Humidity */
#define JEE_SENSOR_CLASS_BARO           (6)  /* Barometer         */
#define JEE_SENSOR_CLASS_LIGHT          (7)  /* Ambient light     */
#define JEE_SENSOR_CLASS_PROXIMITY      (8)  /* Proximity         */
#define JEE_SENSOR_CLASS_HR             (9)  /* Heart Rate        */
#define JEE_SENSOR_CLASS_TVOC           (10) /* TVOC Level        */
#define JEE_SENSOR_CLASS_NOISE          (11) /* Noise Loudness    */
#define JEE_SENSOR_CLASS_STEP           (12) /* Step sensor       */
#define JEE_SENSOR_CLASS_FORCE          (13) /* Force sensor      */
#define JEE_SENSOR_CLASS_DUST           (14) /* Dust sensor       */
#define JEE_SENSOR_CLASS_ECO2           (15) /* eCO2 sensor       */
#define JEE_SENSOR_CLASS_GNSS           (16) /* GPS/GNSS sensor   */
#define JEE_SENSOR_CLASS_TOF            (17) /* TOF sensor        */
#define JEE_SENSOR_CLASS_SPO2           (18) /* SpO2 sensor       */
#define JEE_SENSOR_CLASS_IAQ            (19) /* IAQ sensor.       */
#define JEE_SENSOR_CLASS_ETOH           (20) /* EtOH sensor.      */
#define JEE_SENSOR_CLASS_BP             (21) /* Blood Pressure    */
#define JEE_SENSOR_CLASS_PIR            (22) /* PIR Sensor        */
#define JEE_SENSOR_CLASS_MOTOR          (23) /* Motor Sensor      */
#define JEE_SENSOR_CLASS_SG             (24) /* Steering Gear     */
#define JEE_SENSOR_CLASS_RE             (25) /* Rotary Encoder    */
#define JEE_SENSOR_CLASS_WAPUMP         (26) /* water pump        */
#define JEE_SENSOR_CLASS_SIX_AXIS       (27) /* Six-axis sensor   */
#define JEE_SENSOR_CLASS_RFID           (28) /* Radio Frequency Identification */
#define JEE_SENSOR_CLASS_HYETOMETER     (29) /* Hyetometer sensor */
#define JEE_SENSOR_CLASS_BUTTON_PANEL   (30) /* Button panel      */
#define JEE_SENSOR_CLASS_LEDARRAY       (31) /* led array         */
#define JEE_SENSOR_CLASS_GETSTURE       (32) /* led array         */
#define JEE_SENSOR_CLASS_SCANNER        (33) /* code scanner      */
#define JEE_SENSOR_CLASS_JOYSTICK       (34) /* joystick          */
#define JEE_SENSOR_CLASS_IOEXPAND       (35) /* io expand         */
#define JEE_SENSOR_CLASS_SMOKE          (36) /* smoke             */
#define JEE_SENSOR_CLASS_CO             (37) /* co                */
#define JEE_SENSOR_CLASS_DISPLAY        (38) /* display           */
/* Sensor vendor types */

#define JEE_SENSOR_VENDOR_UNKNOWN       (0)
#define JEE_SENSOR_VENDOR_STM           (1)  /* STMicroelectronics */
#define JEE_SENSOR_VENDOR_BOSCH         (2)  /* Bosch */
#define JEE_SENSOR_VENDOR_INVENSENSE    (3)  /* Invensense */
#define JEE_SENSOR_VENDOR_SEMTECH       (4)  /* Semtech */
#define JEE_SENSOR_VENDOR_GOERTEK       (5)  /* Goertek */
#define JEE_SENSOR_VENDOR_MIRAMEMS      (6)  /* MiraMEMS */
#define JEE_SENSOR_VENDOR_DALLAS        (7)  /* Dallas */
#define JEE_SENSOR_VENDOR_ASAIR         (8)  /* Aosong */
#define JEE_SENSOR_VENDOR_SHARP         (9)  /* Sharp */
#define JEE_SENSOR_VENDOR_SENSIRION     (10) /* Sensirion */
#define JEE_SENSOR_VENDOR_TI            (11) /* Texas Instruments */
#define JEE_SENSOR_VENDOR_PLANTOWER     (12) /* Plantower */
#define JEE_SENSOR_VENDOR_AMS           (13) /* ams AG */
#define JEE_SENSOR_VENDOR_MAXIM         (14) /* Maxim Integrated */
#define JEE_SENSOR_VENDOR_MELEXIS       (15) /* Melexis */

/* Sensor unit types */

#define  JEE_SENSOR_UNIT_NONE           (0)
#define  JEE_SENSOR_UNIT_MG             (1)  /* Accelerometer           unit: mG         */
#define  JEE_SENSOR_UNIT_MDPS           (2)  /* Gyroscope               unit: mdps       */
#define  JEE_SENSOR_UNIT_MGAUSS         (3)  /* Magnetometer            unit: mGauss     */
#define  JEE_SENSOR_UNIT_LUX            (4)  /* Ambient light           unit: lux        */
#define  JEE_SENSOR_UNIT_CM             (5)  /* Distance                unit: cm         */
#define  JEE_SENSOR_UNIT_PA             (6)  /* Barometer               unit: pa         */
#define  JEE_SENSOR_UNIT_PERMILLAGE     (7)  /* Relative Humidity       unit: permillage */
#define  JEE_SENSOR_UNIT_DCELSIUS       (8)  /* Temperature             unit: dCelsius   */
#define  JEE_SENSOR_UNIT_HZ             (9)  /* Frequency               unit: HZ         */
#define  JEE_SENSOR_UNIT_ONE            (10) /* Dimensionless quantity  unit: 1          */
#define  JEE_SENSOR_UNIT_BPM            (11) /* Heart rate              unit: bpm        */
#define  JEE_SENSOR_UNIT_MM             (12) /* Distance                unit: mm         */
#define  JEE_SENSOR_UNIT_MN             (13) /* Force                   unit: mN         */
#define  JEE_SENSOR_UNIT_PPM            (14) /* Concentration           unit: ppm        */
#define  JEE_SENSOR_UNIT_PPB            (15) /* Concentration           unit: ppb        */
#define  JEE_SENSOR_UNIT_DMS            (16) /* Coordinates             unit: DMS        */
#define  JEE_SENSOR_UNIT_DD             (17) /* Coordinates             unit: DD         */
#define  JEE_SENSOR_UNIT_MGM3           (18) /* Concentration           unit: mg/m3      */
#define  JEE_SENSOR_UNIT_MMHG           (19) /* Blood Pressure          unit: mmHg       */
/* Sensor communication interface types */

#define  JEE_SENSOR_INTF_I2C            (1 << 0)
#define  JEE_SENSOR_INTF_SPI            (1 << 1)
#define  JEE_SENSOR_INTF_UART           (1 << 2)
#define  JEE_SENSOR_INTF_ONEWIRE        (1 << 3)

/* Sensor power mode types */

#define  JEE_SENSOR_POWER_NONE          (0)
#define  JEE_SENSOR_POWER_DOWN          (1)  /* power down mode   */
#define  JEE_SENSOR_POWER_NORMAL        (2)  /* normal-power mode */
#define  JEE_SENSOR_POWER_LOW           (3)  /* low-power mode    */
#define  JEE_SENSOR_POWER_HIGH          (4)  /* high-power mode   */

/* Sensor work mode types */

#define  JEE_SENSOR_MODE_NONE           (0)
#define  JEE_SENSOR_MODE_POLLING        (1)  /* One shot only read a data */
#define  JEE_SENSOR_MODE_INT            (2)  /* TODO: One shot interrupt only read a data */
#define  JEE_SENSOR_MODE_FIFO           (3)  /* TODO: One shot interrupt read all fifo data */

/* Sensor control cmd types */

#define  JEE_SENSOR_CTRL_GET_ID         (JEE_DEVICE_CTRL_BASE(Sensor) + 0)  /* Get device id */
#define  JEE_SENSOR_CTRL_GET_INFO       (JEE_DEVICE_CTRL_BASE(Sensor) + 1)  /* Get sensor info */
#define  JEE_SENSOR_CTRL_SET_RANGE      (JEE_DEVICE_CTRL_BASE(Sensor) + 2)  /* Set the measure range of sensor. unit is info of sensor */
#define  JEE_SENSOR_CTRL_SET_ODR        (JEE_DEVICE_CTRL_BASE(Sensor) + 3)  /* Set output date rate. unit is HZ */
#define  JEE_SENSOR_CTRL_SET_MODE       (JEE_DEVICE_CTRL_BASE(Sensor) + 4)  /* Set sensor's work mode. ex. JEE_SENSOR_MODE_POLLING,JEE_SENSOR_MODE_INT */
#define  JEE_SENSOR_CTRL_SET_POWER      (JEE_DEVICE_CTRL_BASE(Sensor) + 5)  /* Set power mode. args type of sensor power mode. ex. JEE_SENSOR_POWER_DOWN,JEE_SENSOR_POWER_NORMAL */
#define  JEE_SENSOR_CTRL_SELF_TEST      (JEE_DEVICE_CTRL_BASE(Sensor) + 6)  /* Take a self test */

#define  JEE_SENSOR_CTRL_USER_CMD_START 0x100  /* User commands should be greater than 0x100 */

struct jee_sensor_info
{
    jee_uint8_t     type;                    /* The sensor type */
    jee_uint8_t     vendor;                  /* Vendor of sensors */
    const char      *model;                  /* model name of sensor */
    jee_uint8_t     unit;                    /* unit of measurement */
    jee_uint8_t     intf_type;               /* Communication interface type */
    jee_int32_t     range_max;               /* maximum range of this sensor's value. unit is 'unit' */
    jee_int32_t     range_min;               /* minimum range of this sensor's value. unit is 'unit' */
    jee_uint32_t    period_min;              /* Minimum measurement period,unit:ms. zero = not a constant rate */
    jee_uint8_t     fifo_max;
};

struct jee_sensor_intf
{
    char                       *dev_name;   /* The name of the communication device */
    jee_uint8_t                 type;       /* Communication interface type */
    void                       *user_data;  /* Private data for the sensor. ex. i2c addr,spi cs,control I/O */
};

struct jee_sensor_config
{
    struct jee_sensor_intf        intf;      /* sensor interface config */
    struct jee_device_pin_mode    irq_pin;   /* Interrupt pin, The purpose of this pin is to notification read data */
    jee_uint8_t                   mode;      /* sensor work mode */
    jee_uint8_t                   power;     /* sensor power mode */
    jee_uint16_t                  odr;       /* sensor out data rate */
    jee_int32_t                   range;     /* sensor range of measurement */
};

typedef struct jee_sensor_device *jee_sensor_t;

struct jee_sensor_device
{
    struct jee_device             parent;    /* The standard device */

    struct jee_sensor_info        info;      /* The sensor info data */
    struct jee_sensor_config      config;    /* The sensor config data */

    void                         *data_buf;  /* The buf of the data received */
    jee_size_t                    data_len;  /* The size of the data received */

    const struct jee_sensor_ops  *ops;       /* The sensor ops */

    struct jee_sensor_module     *module;    /* The sensor module */

    jee_err_t (*irq_handle)(jee_sensor_t sensor);             /* Called when an interrupt is generated, registered by the driver */
};

struct jee_sensor_module
{
    //jee_mutex_t                   lock;                      /* The module lock */todo

    jee_sensor_t                  sen[JEE_SENSOR_MODULE_MAX]; /* The module contains a list of sensors */
    jee_uint8_t                   sen_num;                   /* Number of sensors contained in the module */
};

/* 3-axis Data Type */
struct sensor_3_axis
{
    jee_int32_t x;
    jee_int32_t y;
    jee_int32_t z;
};

/* Blood Pressure Data Type */
struct sensor_bp
{
    jee_int32_t sbp; /* SBP : systolic pressure */
    jee_int32_t dbp; /* DBP : diastolic pressure */
};

struct coordinates
{
    double longitude;
    double latitude;
};

struct jee_sensor_data
{
    jee_uint32_t         timestamp;          /* The timestamp when the data was received */
    jee_uint8_t          type;               /* The sensor type of the data */
    union
    {
        struct sensor_3_axis acce;          /* Accelerometer.       unit: mG          */
        struct sensor_3_axis gyro;          /* Gyroscope.           unit: mdps        */
        struct sensor_3_axis mag;           /* Magnetometer.        unit: mGauss      */
        struct coordinates   coord;         /* Coordinates          unit: degrees     */
        jee_int32_t           temp;          /* Temperature.         unit: dCelsius    */
        jee_int32_t           humi;          /* Relative humidity.   unit: permillage  */
        jee_int32_t           baro;          /* Pressure.            unit: pascal (Pa) */
        jee_int32_t           light;         /* Light.               unit: lux         */
        jee_int32_t           proximity;     /* Distance.            unit: centimeters */
        jee_int32_t           hr;            /* Heart rate.          unit: bpm         */
        jee_int32_t           tvoc;          /* TVOC.                unit: permillage  */
        jee_int32_t           noise;         /* Noise Loudness.      unit: HZ          */
        jee_uint32_t          step;          /* Step sensor.         unit: 1           */
        jee_int32_t           force;         /* Force sensor.        unit: mN          */
        jee_uint32_t          dust;          /* Dust sensor.         unit: ug/m3       */
        jee_uint32_t          eco2;          /* eCO2 sensor.         unit: ppm         */
        jee_uint32_t          spo2;          /* SpO2 sensor.         unit: permillage  */
        jee_uint32_t          iaq;           /* IAQ sensor.          unit: 1 */
        jee_uint32_t          etoh;          /* EtOH sensor.         unit: ppm */
        struct sensor_bp     bp;            /* BloodPressure.       unit: mmHg        */
    } data;
};

struct jee_sensor_ops
{
    jee_size_t (*fetch_data)(struct jee_sensor_device *sensor, void *buf, jee_size_t len);
    jee_err_t (*control)(struct jee_sensor_device *sensor, int cmd, void *arg);
};





int jee_hw_sensor_register(jee_sensor_t      sensor,
                          const char        *name,
                          jee_uint32_t       flag,
                          void              *data);


extern char *const sensor_name_str;


#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_H__ */

