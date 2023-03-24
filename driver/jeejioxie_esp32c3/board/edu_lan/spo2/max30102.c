#include "max30102.h"
#include <stdio.h>
#include "esp32c3/rom/ets_sys.h" //for ets_delay_us()
#include "hal_pwm.h"
#include "hal_i2c.h"
#include "hal_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#define I2C_MASTER_SCL_IO 9         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 8         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define GPIO_INT_INPUT 2
#define GPIO_INT_INPUT_PIN_SEL ((1ULL << GPIO_INT_INPUT))

#define GPIO_led_OUTPUT 4
#define GPIO_led_OUTPUT_PIN_SEL ((1ULL << GPIO_led_OUTPUT))

#define PWM_TIMER LEDC_TIMER_0
#define PWM_MODE LEDC_LOW_SPEED_MODE /*!< LEDC high speed speed_mode */
#define PWM_OUTPUT_IO (5)            // Define the output GPIO
#define PWM_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT
#define PWM_DUTY (4095)      // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define PWM_FREQUENCY (5000) // Frequency in Hertz. Set frequency at 5 kHz
#define TARGET_MCPWM_UNIT MCPWM_UNIT_0

#define MAX_BRIGHTNESS 255
////////////////////////////////////////////
#define I2C_WRITE_ADDR 0x57
#define I2C_READ_ADDR 0x57

// register addresses
#define REG_INTR_STATUS_1 0x00
#define REG_INTR_STATUS_2 0x01
#define REG_INTR_ENABLE_1 0x02
#define REG_INTR_ENABLE_2 0x03
#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_COUNTER 0x05
#define REG_FIFO_RD_PTR 0x06
#define REG_FIFO_DATA 0x07
#define REG_FIFO_CONFIG 0x08
#define REG_MODE_CONFIG 0x09
#define REG_SPO2_CONFIG 0x0A
#define REG_LED1_PA 0x0C
#define REG_LED2_PA 0x0D
#define REG_PILOT_PA 0x10
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12
#define REG_TEMP_INTR 0x1F
#define REG_TEMP_FRAC 0x20
#define REG_TEMP_CONFIG 0x21
#define REG_PROX_INT_THRESH 0x30
#define REG_REV_ID 0xFE
#define REG_PART_ID 0xFF
//////////////////////////////////////////////////
uint32_t aun_ir_buffer[500];  // IR LED sensor data
int32_t n_ir_buffer_length;   // data length
uint32_t aun_red_buffer[500]; // Red LED sensor data
int32_t n_sp02;               // SPO2 value
int8_t ch_spo2_valid;         // indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;         // heart rate value
int8_t ch_hr_valid;           // indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;

int32_t spo2_res = 0;
int32_t heart_rate_res = 0;

uint8_t start_flag = 0;
uint8_t send_flag = 0;

#define FS 100
#define BUFFER_SIZE (FS * 5)
#define HR_FIFO_SIZE 7
#define MA4_SIZE 4     // DO NOT CHANGE
#define HAMMING_SIZE 5 // DO NOT CHANGE
#define min(x, y) ((x) < (y) ? (x) : (y))

static jee_device_t pwmDev = NULL;
static struct jee_pwm_configuration pwm_cfg;
static jee_device_t max30102IICDev;

void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, int8_t *pch_hr_valid);
void maxim_find_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num);
void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_size, int32_t n_min_height);
void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance);
void maxim_sort_ascend(int32_t *pn_x, int32_t n_size);
void maxim_sort_indices_descend(int32_t *pn_x, int32_t *pn_indx, int32_t n_size);

int i2c_write(uint8_t device_address, uint8_t *write_buffer, size_t write_size, bool ff);
int i2c_read(uint8_t device_address, uint8_t reg_addr, uint8_t *read_buffer, size_t read_size, bool ff); //(I2C_READ_ADDR, &ch_i2c_data, 1, false)==0)
//////////////////////////////////////////////////////
bool maxim_max30102_write_reg(uint8_t uch_addr, uint8_t uch_data)
/**
 * \brief        Write a value to a MAX30102 register
 * \par          Details
 *               This function writes a value to a MAX30102 register
 *
 * \param[in]    uch_addr    - register address
 * \param[in]    uch_data    - register data
 *
 * \retval       true on success
 */
{
#if 1
    uint8_t ach_i2c_data[2];
    ach_i2c_data[0] = uch_addr;
    ach_i2c_data[1] = uch_data;

    if (i2c_write(I2C_WRITE_ADDR, ach_i2c_data, 2, false) == 0)
        return true;
    else
#endif
        return false;
}

bool maxim_max30102_read_reg(uint8_t uch_addr, uint8_t *puch_data)
/**
 * \brief        Read a MAX30102 register
 * \par          Details
 *               This function reads a MAX30102 register
 *
 * \param[in]    uch_addr    - register address
 * \param[out]   puch_data    - pointer that stores the register data
 *
 * \retval       true on success
 */
{
    uint8_t ch_i2c_data;

    if (i2c_read(I2C_READ_ADDR, uch_addr, &ch_i2c_data, 1, false) == 0)
    {
        *puch_data = (uint8_t)ch_i2c_data;
        return true;
    }
    else
#if 0
  uint8_t ch_i2c_data;
  ch_i2c_data=uch_addr;
  if(i2c_write(I2C_WRITE_ADDR, &ch_i2c_data, 1, true)!=0)
    return false;
  if(i2c_read(I2C_READ_ADDR, &ch_i2c_data, 1, false)==0)
  {
    *puch_data=(uint8_t) ch_i2c_data;
    return true;
  }
  else
#endif
        return false;
}

bool maxim_max30102_init()
/**
 * \brief        Initialize the MAX30102
 * \par          Details
 *               This function initializes the MAX30102
 *
 * \param        None
 *
 * \retval       true on success
 */
{
#if 1
    if (!maxim_max30102_write_reg(REG_INTR_ENABLE_1, 0xc0)) // INTR setting
        return false;
    if (!maxim_max30102_write_reg(REG_INTR_ENABLE_2, 0x00))
        return false;
    if (!maxim_max30102_write_reg(REG_FIFO_WR_PTR, 0x00)) // FIFO_WR_PTR[4:0]
        return false;
    if (!maxim_max30102_write_reg(REG_OVF_COUNTER, 0x00)) // OVF_COUNTER[4:0]
        return false;
    if (!maxim_max30102_write_reg(REG_FIFO_RD_PTR, 0x00)) // FIFO_RD_PTR[4:0]
        return false;
    if (!maxim_max30102_write_reg(REG_FIFO_CONFIG, 0x0f)) // sample avg = 1, fifo rollover=false, fifo almost full = 17
        return false;
    if (!maxim_max30102_write_reg(REG_MODE_CONFIG, 0x03)) // 0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
        return false;
    if (!maxim_max30102_write_reg(REG_SPO2_CONFIG, 0x27)) // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)
        return false;

    if (!maxim_max30102_write_reg(REG_LED1_PA, 0x24)) // Choose value for ~ 7mA for LED1
        return false;
    if (!maxim_max30102_write_reg(REG_LED2_PA, 0x24)) // Choose value for ~ 7mA for LED2
        return false;
    if (!maxim_max30102_write_reg(REG_PILOT_PA, 0x7f)) // Choose value for ~ 25mA for Pilot LED
        return false;
#endif
    return true;
}

bool maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
/**
 * \brief        Read a set of samples from the MAX30102 FIFO register
 * \par          Details
 *               This function reads a set of samples from the MAX30102 FIFO register
 *
 * \param[out]   *pun_red_led   - pointer that stores the red LED reading data
 * \param[out]   *pun_ir_led    - pointer that stores the IR LED reading data
 *
 * \retval       true on success
 */
{
#if 1
    uint32_t un_temp;
    unsigned char uch_temp;
    *pun_red_led = 0;
    *pun_ir_led = 0;
    uint8_t ach_i2c_data[6];

    // read and clear status register
    maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_temp);
    maxim_max30102_read_reg(REG_INTR_STATUS_2, &uch_temp);

    // ach_i2c_data[0]=REG_FIFO_DATA;
    if (i2c_read(I2C_READ_ADDR, REG_FIFO_DATA, ach_i2c_data, 6, false) != 0)
    {
        return false;
    }

    /*
  if(i2c_write(I2C_WRITE_ADDR, ach_i2c_data, 1, true)!=0)
    return false;
  if(i2c_read(I2C_READ_ADDR, ach_i2c_data, 6, false)!=0)
  {
    return false;
  }*/
    un_temp = (unsigned char)ach_i2c_data[0];
    un_temp <<= 16;
    *pun_red_led += un_temp;
    un_temp = (unsigned char)ach_i2c_data[1];
    un_temp <<= 8;
    *pun_red_led += un_temp;
    un_temp = (unsigned char)ach_i2c_data[2];
    *pun_red_led += un_temp;

    un_temp = (unsigned char)ach_i2c_data[3];
    un_temp <<= 16;
    *pun_ir_led += un_temp;
    un_temp = (unsigned char)ach_i2c_data[4];
    un_temp <<= 8;
    *pun_ir_led += un_temp;
    un_temp = (unsigned char)ach_i2c_data[5];
    *pun_ir_led += un_temp;
    *pun_red_led &= 0x03FFFF; // Mask MSB [23:18]
    *pun_ir_led &= 0x03FFFF;  // Mask MSB [23:18]

#endif
    return true;
}

bool maxim_max30102_reset()
/**
 * \brief        Reset the MAX30102
 * \par          Details
 *               This function resets the MAX30102
 *
 * \param        None
 *
 * \retval       true on success
 */
{
#if 1
    if (!maxim_max30102_write_reg(REG_MODE_CONFIG, 0x40))
        return false;
    else
#endif
    {
        if (!maxim_max30102_write_reg(REG_MODE_CONFIG, 0x40))
            return false;
        else
            return true;
    }
}
///////////////////////////////////////////////////////

const uint16_t auw_hamm[31] = {41, 276, 512, 276, 41}; // Hamm=  long16(512* hamming(5)');
// uch_spo2_table is computed as  -45.060*ratioAverage* ratioAverage + 30.354 *ratioAverage + 94.845 ;
const uint8_t uch_spo2_table[184] = {95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
                                     99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
                                     100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
                                     97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
                                     90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
                                     80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
                                     66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
                                     49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
                                     28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
                                     3, 2, 1};

static int32_t an_dx[BUFFER_SIZE - MA4_SIZE]; // delta
static int32_t an_x[BUFFER_SIZE];             // ir
static int32_t an_y[BUFFER_SIZE];             // red

void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid,
                                            int32_t *pn_heart_rate, int8_t *pch_hr_valid)
/**
 * \brief        Calculate the heart rate and SpO2 level
 * \par          Details
 *               By detecting  peaks of PPG cycle and corresponding AC/DC of red/infra-red signal, the ratio for the SPO2 is computed.
 *               Since this algorithm is aiming for Arm M0/M3. formaula for SPO2 did not achieve the accuracy due to register overflow.
 *               Thus, accurate SPO2 is precalculated and save longo uch_spo2_table[] per each ratio.
 *
 * \param[in]    *pun_ir_buffer           - IR sensor data buffer
 * \param[in]    n_ir_buffer_length      - IR sensor data buffer length
 * \param[in]    *pun_red_buffer          - Red sensor data buffer
 * \param[out]    *pn_spo2                - Calculated SpO2 value
 * \param[out]    *pch_spo2_valid         - 1 if the calculated SpO2 value is valid
 * \param[out]    *pn_heart_rate          - Calculated heart rate value
 * \param[out]    *pch_hr_valid           - 1 if the calculated heart rate value is valid
 *
 * \retval       None
 */
{
    uint32_t un_ir_mean, un_only_once;
    int32_t k, n_i_ratio_count;
    int32_t i, s, m, n_exact_ir_valley_locs_count, n_middle_idx;
    int32_t n_th1, n_npks, n_c_min;
    int32_t an_ir_valley_locs[15];
    int32_t an_exact_ir_valley_locs[15];
    int32_t an_dx_peak_locs[15];
    int32_t n_peak_interval_sum;

    int32_t n_y_ac, n_x_ac;
    int32_t n_spo2_calc;
    int32_t n_y_dc_max, n_x_dc_max;
    int32_t n_y_dc_max_idx = 0, n_x_dc_max_idx = 0;
    int32_t an_ratio[5], n_ratio_average;
    int32_t n_nume, n_denom;
    // remove DC of ir signal
    un_ir_mean = 0;
    for (k = 0; k < n_ir_buffer_length; k++)
        un_ir_mean += pun_ir_buffer[k];
    un_ir_mean = un_ir_mean / n_ir_buffer_length;
    for (k = 0; k < n_ir_buffer_length; k++)
        an_x[k] = pun_ir_buffer[k] - un_ir_mean;

    // 4 pt Moving Average
    for (k = 0; k < BUFFER_SIZE - MA4_SIZE; k++)
    {
        n_denom = (an_x[k] + an_x[k + 1] + an_x[k + 2] + an_x[k + 3]);
        an_x[k] = n_denom / (int32_t)4;
    }

    // get difference of smoothed IR signal

    for (k = 0; k < BUFFER_SIZE - MA4_SIZE - 1; k++)
        an_dx[k] = (an_x[k + 1] - an_x[k]);

    // 2-pt Moving Average to an_dx
    for (k = 0; k < BUFFER_SIZE - MA4_SIZE - 2; k++)
    {
        an_dx[k] = (an_dx[k] + an_dx[k + 1]) / 2;
    }

    // hamming window
    // flip wave form so that we can detect valley with peak detector
    for (i = 0; i < BUFFER_SIZE - HAMMING_SIZE - MA4_SIZE - 2; i++)
    {
        s = 0;
        for (k = i; k < i + HAMMING_SIZE; k++)
        {
            s -= an_dx[k] * auw_hamm[k - i];
        }
        an_dx[i] = s / (int32_t)1146; // divide by sum of auw_hamm
    }

    n_th1 = 0; // threshold calculation
    for (k = 0; k < BUFFER_SIZE - HAMMING_SIZE; k++)
    {
        n_th1 += ((an_dx[k] > 0) ? an_dx[k] : ((int32_t)0 - an_dx[k]));
    }
    n_th1 = n_th1 / (BUFFER_SIZE - HAMMING_SIZE);
    // peak location is acutally index for sharpest location of raw signal since we flipped the signal
    maxim_find_peaks(an_dx_peak_locs, &n_npks, an_dx, BUFFER_SIZE - HAMMING_SIZE, n_th1, 8, 5); // peak_height, peak_distance, max_num_peaks

    n_peak_interval_sum = 0;
    if (n_npks >= 2)
    {
        for (k = 1; k < n_npks; k++)
            n_peak_interval_sum += (an_dx_peak_locs[k] - an_dx_peak_locs[k - 1]);
        n_peak_interval_sum = n_peak_interval_sum / (n_npks - 1);
        *pn_heart_rate = (int32_t)(6000 / n_peak_interval_sum); // beats per minutes
        *pch_hr_valid = 1;
    }
    else
    {
        *pn_heart_rate = -999;
        *pch_hr_valid = 0;
    }

    for (k = 0; k < n_npks; k++)
        an_ir_valley_locs[k] = an_dx_peak_locs[k] + HAMMING_SIZE / 2;

    // raw value : RED(=y) and IR(=X)
    // we need to assess DC and AC value of ir and red PPG.
    for (k = 0; k < n_ir_buffer_length; k++)
    {
        an_x[k] = pun_ir_buffer[k];
        an_y[k] = pun_red_buffer[k];
    }

    // find precise min near an_ir_valley_locs
    n_exact_ir_valley_locs_count = 0;
    for (k = 0; k < n_npks; k++)
    {
        un_only_once = 1;
        m = an_ir_valley_locs[k];
        n_c_min = 16777216; // 2^24;
        if (m + 5 < BUFFER_SIZE - HAMMING_SIZE && m - 5 > 0)
        {
            for (i = m - 5; i < m + 5; i++)
                if (an_x[i] < n_c_min)
                {
                    if (un_only_once > 0)
                    {
                        un_only_once = 0;
                    }
                    n_c_min = an_x[i];
                    an_exact_ir_valley_locs[k] = i;
                }
            if (un_only_once == 0)
                n_exact_ir_valley_locs_count++;
        }
    }
    if (n_exact_ir_valley_locs_count < 2)
    {
        *pn_spo2 = -999; // do not use SPO2 since signal ratio is out of range
        *pch_spo2_valid = 0;
        return;
    }
    // 4 pt MA
    for (k = 0; k < BUFFER_SIZE - MA4_SIZE; k++)
    {
        an_x[k] = (an_x[k] + an_x[k + 1] + an_x[k + 2] + an_x[k + 3]) / (int32_t)4;
        an_y[k] = (an_y[k] + an_y[k + 1] + an_y[k + 2] + an_y[k + 3]) / (int32_t)4;
    }

    // using an_exact_ir_valley_locs , find ir-red DC andir-red AC for SPO2 calibration ratio
    // finding AC/DC maximum of raw ir * red between two valley locations
    n_ratio_average = 0;
    n_i_ratio_count = 0;

    for (k = 0; k < 5; k++)
        an_ratio[k] = 0;
    for (k = 0; k < n_exact_ir_valley_locs_count; k++)
    {
        if (an_exact_ir_valley_locs[k] > BUFFER_SIZE)
        {
            *pn_spo2 = -999; // do not use SPO2 since valley loc is out of range
            *pch_spo2_valid = 0;
            return;
        }
    }
    // find max between two valley locations
    // and use ratio betwen AC compoent of Ir & Red and DC compoent of Ir & Red for SPO2

    for (k = 0; k < n_exact_ir_valley_locs_count - 1; k++)
    {
        n_y_dc_max = -16777216;
        n_x_dc_max = -16777216;
        if (an_exact_ir_valley_locs[k + 1] - an_exact_ir_valley_locs[k] > 10)
        {
            for (i = an_exact_ir_valley_locs[k]; i < an_exact_ir_valley_locs[k + 1]; i++)
            {
                if (an_x[i] > n_x_dc_max)
                {
                    n_x_dc_max = an_x[i];
                    n_x_dc_max_idx = i;
                }
                if (an_y[i] > n_y_dc_max)
                {
                    n_y_dc_max = an_y[i];
                    n_y_dc_max_idx = i;
                }
            }
            n_y_ac = (an_y[an_exact_ir_valley_locs[k + 1]] - an_y[an_exact_ir_valley_locs[k]]) * (n_y_dc_max_idx - an_exact_ir_valley_locs[k]); // red
            n_y_ac = an_y[an_exact_ir_valley_locs[k]] + n_y_ac / (an_exact_ir_valley_locs[k + 1] - an_exact_ir_valley_locs[k]);

            n_y_ac = an_y[n_y_dc_max_idx] - n_y_ac;                                                                                             // subracting linear DC compoenents from raw
            n_x_ac = (an_x[an_exact_ir_valley_locs[k + 1]] - an_x[an_exact_ir_valley_locs[k]]) * (n_x_dc_max_idx - an_exact_ir_valley_locs[k]); // ir
            n_x_ac = an_x[an_exact_ir_valley_locs[k]] + n_x_ac / (an_exact_ir_valley_locs[k + 1] - an_exact_ir_valley_locs[k]);
            n_x_ac = an_x[n_y_dc_max_idx] - n_x_ac; // subracting linear DC compoenents from raw
            n_nume = (n_y_ac * n_x_dc_max) >> 7;    // prepare X100 to preserve floating value
            n_denom = (n_x_ac * n_y_dc_max) >> 7;
            if (n_denom > 0 && n_i_ratio_count < 5 && n_nume != 0)
            {
                an_ratio[n_i_ratio_count] = (n_nume * 20) / n_denom; // formular is ( n_y_ac *n_x_dc_max) / ( n_x_ac *n_y_dc_max) ;  ///*************************n_numeÔ­À´ÊÇ*100************************//
                n_i_ratio_count++;
            }
        }
    }

    maxim_sort_ascend(an_ratio, n_i_ratio_count);
    n_middle_idx = n_i_ratio_count / 2;

    if (n_middle_idx > 1)
        n_ratio_average = (an_ratio[n_middle_idx - 1] + an_ratio[n_middle_idx]) / 2; // use median
    else
        n_ratio_average = an_ratio[n_middle_idx];

    if (n_ratio_average > 2 && n_ratio_average < 184)
    {
        n_spo2_calc = uch_spo2_table[n_ratio_average];
        *pn_spo2 = n_spo2_calc;
        *pch_spo2_valid = 1; //  float_SPO2 =  -45.060*n_ratio_average* n_ratio_average/10000 + 30.354 *n_ratio_average/100 + 94.845 ;  // for comparison with table
    }
    else
    {
        *pn_spo2 = -999; // do not use SPO2 since signal ratio is out of range
        *pch_spo2_valid = 0;
    }
}

void maxim_find_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num)
/**
 * \brief        Find peaks
 * \par          Details
 *               Find at most MAX_NUM peaks above MIN_HEIGHT separated by at least MIN_DISTANCE
 *
 * \retval       None
 */
{
#if 1
    maxim_peaks_above_min_height(pn_locs, pn_npks, pn_x, n_size, n_min_height);
    maxim_remove_close_peaks(pn_locs, pn_npks, pn_x, n_min_distance);
    *pn_npks = min(*pn_npks, n_max_num);
#endif
}

void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_size, int32_t n_min_height)
/**
 * \brief        Find peaks above n_min_height
 * \par          Details
 *               Find all peaks above MIN_HEIGHT
 *
 * \retval       None
 */
{
    int32_t i = 1, n_width;
    *pn_npks = 0;

    while (i < n_size - 1)
    {
        if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i - 1])
        { // find left edge of potential peaks
            n_width = 1;
            while (i + n_width < n_size && pn_x[i] == pn_x[i + n_width]) // find flat peaks
                n_width++;
            if (pn_x[i] > pn_x[i + n_width] && (*pn_npks) < 15)
            { // find right edge of peaks
                pn_locs[(*pn_npks)++] = i;
                // for flat peaks, peak location is left edge
                i += n_width + 1;
            }
            else
                i += n_width;
        }
        else
            i++;
    }
}

void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance)
/**
 * \brief        Remove peaks
 * \par          Details
 *               Remove peaks separated by less than MIN_DISTANCE
 *
 * \retval       None
 */
{
    int32_t i, j, n_old_npks, n_dist;

    /* Order peaks from large to small */
    maxim_sort_indices_descend(pn_x, pn_locs, *pn_npks);

    for (i = -1; i < *pn_npks; i++)
    {
        n_old_npks = *pn_npks;
        *pn_npks = i + 1;
        for (j = i + 1; j < n_old_npks; j++)
        {
            n_dist = pn_locs[j] - (i == -1 ? -1 : pn_locs[i]); // lag-zero peak of autocorr is at index -1
            if (n_dist > n_min_distance || n_dist < -n_min_distance)
                pn_locs[(*pn_npks)++] = pn_locs[j];
        }
    }

    // Resort indices longo ascending order
    maxim_sort_ascend(pn_locs, *pn_npks);
}

void maxim_sort_ascend(int32_t *pn_x, int32_t n_size)
/**
 * \brief        Sort array
 * \par          Details
 *               Sort array in ascending order (insertion sort algorithm)
 *
 * \retval       None
 */
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++)
    {
        n_temp = pn_x[i];
        for (j = i; j > 0 && n_temp < pn_x[j - 1]; j--)
            pn_x[j] = pn_x[j - 1];
        pn_x[j] = n_temp;
    }
}

void maxim_sort_indices_descend(int32_t *pn_x, int32_t *pn_indx, int32_t n_size)
/**
 * \brief        Sort indices
 * \par          Details
 *               Sort indices according to descending order (insertion sort algorithm)
 *
 * \retval       None
 */
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++)
    {
        n_temp = pn_indx[i];
        for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j - 1]]; j--)
            pn_indx[j] = pn_indx[j - 1];
        pn_indx[j] = n_temp;
    }
}
///////////////////////////////////////////////////////

static esp_err_t i2c_master_init(void)
{

    max30102IICDev = jee_device_find("i2c0hard");
    if (max30102IICDev == JEE_NULL)
    {
        printf("can not find %s Model\n", "i2c0hard");
        return ESP_FAIL;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(max30102IICDev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open %s device\n", "i2c0hard");
        return ESP_FAIL;
    }

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
}

void int_gpio_init(void)
{
    jee_pin_mode(GPIO_INT_INPUT, PIN_MODE_INPUT);
    jee_pin_mode(GPIO_led_OUTPUT, PIN_MODE_OUTPUT);

#if 0
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INT_INPUT_PIN_SEL;
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    // enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    // bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_led_OUTPUT_PIN_SEL;
    // set as input mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // enable pull-up mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
#endif
}
void led_pwm_init()
{
#if 1
    pwmDev = jee_device_find("pwm");
    if (pwmDev == JEE_NULL)
    {
        printf("can not find pwm Model\n");
        return;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(pwmDev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open pin device\n");
        return;
    }

    pwm_cfg.channel = PWM_CHANNEL;
    pwm_cfg.timer_num = PWM_TIMER;
    pwm_cfg.duty = 0;
    pwm_cfg.period = (1e9) / PWM_FREQUENCY;
    pwm_cfg.pin_num = PWM_OUTPUT_IO;
    pwm_cfg.speed_mode = PWM_MODE;
    // 配置设备
    jee_device_control(pwmDev, PWM_CMD_INIT, (void *)&pwm_cfg);
#endif
#if 0
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = PWM_FREQUENCY, // Set output frequency at 5 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL,
        .timer_sel = PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PWM_OUTPUT_IO,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
#endif
}

int i2c_read(uint8_t device_address, uint8_t reg_addr, uint8_t *read_buffer, size_t read_size, bool ff) //(I2C_READ_ADDR, &ch_i2c_data, 1, false)==0)
{

    jee_i2c_master_send(max30102IICDev, device_address, JEE_I2C_STOP, &reg_addr, 1);
    jee_i2c_master_recv(max30102IICDev, device_address, JEE_I2C_STOP, read_buffer, read_size);

    return 0;

    // return i2c_master_write_read_device(I2C_MASTER_NUM, device_address, &reg_addr, 1, read_buffer, read_size, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    //  return i2c_master_read_from_device(I2C_MASTER_NUM, device_address,read_buffer,read_size, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}
int i2c_write(uint8_t device_address, uint8_t *write_buffer, size_t write_size, bool ff)
{

    jee_i2c_master_send(max30102IICDev, device_address, JEE_I2C_STOP, write_buffer, write_size);
    return 0;

    // return i2c_master_write_to_device(I2C_MASTER_NUM, device_address, write_buffer, write_size, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

void pwmled_write(float dut)
{

    pwm_cfg.duty = 8191 * dut;
    jee_device_control(pwmDev, PWM_CMD_SET_DUTY, (void *)&pwm_cfg);
    jee_device_control(pwmDev, PWM_CMD_UPDATE_DUTY, (void *)&pwm_cfg);

#if 0
    //  printf("pwmled_write %d\n",du);
    // vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL, pwm_cfg.duty));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL));
#endif
}
static void max30102_task_example(void *arg)
{

    uint32_t un_min, un_max, un_prev_data; // variables to calculate the on-board LED brightness that reflects the heartbeats
    int i;
    int32_t n_brightness;
    float f_temp;
    // printf("max30102_task_example1\n\r");
    maxim_max30102_reset(); // resets the MAX30102
    vTaskDelay(pdMS_TO_TICKS(50));
    //  maxim_max30102_read_reg(0, &uch_dummy);
    // printf("max30102_task_example2\n\r");
    maxim_max30102_init();
    // printf("max30102_task_example3\n\r");
    n_brightness = 0;
    un_min = 0x3FFFF;
    un_max = 0;

    n_ir_buffer_length = 500;
    uint8_t k = 0;
    // read the first 500 samples, and determine the signal range
    for (i = 0; i < n_ir_buffer_length; i++)
    {
        k = 0;
        while (jee_pin_read(GPIO_INT_INPUT) == 1)
        {
            if (k++ > 100)
            {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }                                                                    // wait until the interrupt pin asserts
        maxim_max30102_read_fifo((aun_red_buffer + i), (aun_ir_buffer + i)); // read from MAX30102 FIFO
        if (aun_red_buffer[i] == 0 && aun_ir_buffer[i] == 0)
        {
            break;
        }
        if (un_min > aun_red_buffer[i])
            un_min = aun_red_buffer[i]; // update signal min
        if (un_max < aun_red_buffer[i])
            un_max = aun_red_buffer[i]; // update signal max
#if 0
        printf("red=");
        printf("%ld", aun_red_buffer[i]);
        printf(", ir=");
        printf("%ld\n\r", aun_ir_buffer[i]);
#endif
    }

    un_prev_data = aun_red_buffer[i];

    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);

    // Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
    while (1)
    {
        i = 0;
        un_min = 0x3FFFF;
        un_max = 0;

        if (start_flag == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
        for (i = 100; i < 500; i++)
        {
            aun_red_buffer[i - 100] = aun_red_buffer[i];
            aun_ir_buffer[i - 100] = aun_ir_buffer[i];

            // update the signal min and max
            if (un_min > aun_red_buffer[i])
                un_min = aun_red_buffer[i];
            if (un_max < aun_red_buffer[i])
                un_max = aun_red_buffer[i];
        }

        // take 100 sets of samples before calculating the heart rate.
        for (i = 400; i < 500; i++)
        {
            un_prev_data = aun_red_buffer[i - 1];
            while (jee_pin_read(GPIO_INT_INPUT) == 1)
            {
                vTaskDelay(pdMS_TO_TICKS(5));
            }
            maxim_max30102_read_fifo((aun_red_buffer + i), (aun_ir_buffer + i));
            // printf("aun_red_buffer[%d] %d\n",i,aun_red_buffer[i] );
            if (aun_red_buffer[i] > un_prev_data) // just to determine the brightness of LED according to the deviation of adjacent two AD data
            {
                f_temp = aun_red_buffer[i] - un_prev_data;
                f_temp /= (un_max - un_min);
                f_temp *= MAX_BRIGHTNESS;
                n_brightness -= (int)f_temp;
                if (n_brightness < 0)
                    n_brightness = 0;
            }
            else
            {
                f_temp = un_prev_data - aun_red_buffer[i];
                f_temp /= (un_max - un_min);
                f_temp *= MAX_BRIGHTNESS;
                n_brightness += (int)f_temp;
                if (n_brightness > MAX_BRIGHTNESS)
                    n_brightness = MAX_BRIGHTNESS;
            }
#if 1
            pwmled_write(1 - (float)n_brightness / 256); // pwm control led brightness
            if (n_brightness < 120)
                jee_pin_write(GPIO_led_OUTPUT, 1);
            else
                jee_pin_write(GPIO_led_OUTPUT, 0);
#endif
// send samples and calculation result to terminal program through UART
#if 0
            printf("red=");
            printf("%ld", aun_red_buffer[i]);
            printf(", ir=");
            printf("%ld", aun_ir_buffer[i]);
            printf(", HR=%ld, ", n_heart_rate); 
            printf("HRvalid=%d, ", ch_hr_valid);
            printf("SpO2=%ld, ", n_sp02);
            printf("SPO2Valid=%d\n\r", ch_spo2_valid);
#endif
        }
        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
        printf("saturation %ld:", aun_ir_buffer[i]);
        printf("HR=%ld, ", n_heart_rate);
        printf("HRvalid=%d, ", ch_hr_valid);
        printf("SpO2=%ld, ", n_sp02);
        printf("SPO2Valid=%d\n\r", ch_spo2_valid);
        if (ch_hr_valid == 1 && n_heart_rate > 120)
        {
            n_heart_rate = 0;
            n_sp02 = 0;
        }
        spo2_res = n_sp02;
        heart_rate_res = n_heart_rate;

        send_flag = 1;
    }
}

void user_max30102_init(void)
{

    i2c_master_init();
    int_gpio_init();
    led_pwm_init();
    printf("--------------------user_max30102_init--------------------------\n");
    xTaskCreate(max30102_task_example, "max30102_task_example", 3048, NULL, 10, NULL);
}

void bloodoxygen_sensor_init(void)
{
    user_max30102_init();
}

void bloodoxygen_sensor_get_data(int *n_sp02, int *n_heart_rate)
{
    if (heart_rate_res == -999)
        heart_rate_res = 0;
    if (spo2_res == -999)
        spo2_res = 0;
    *n_sp02 = spo2_res;
    *n_heart_rate = heart_rate_res;
}

uint8_t bloodoxygen_get_onoff(void)
{
    return start_flag;
}
uint8_t bloodoxygen_get_send_flag(void)
{
    if (send_flag == 1)
    {
        send_flag = 0;
        return 1;
    }
    return 0;
}
void bloodoxygen_set_onoff(uint8_t flag)
{
    start_flag = flag;
}
