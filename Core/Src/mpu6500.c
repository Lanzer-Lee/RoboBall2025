//
// Created by lanzer on 24-12-6.
//

#include "mpu6500.h"
#include "string.h"
#include <math.h>

volatile float q0 = 1.0f;
volatile float q1 = 0.0f;
volatile float q2 = 0.0f;
volatile float q3 = 0.0f;
volatile float exInt, eyInt, ezInt;                   /* error integral */
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;
volatile uint32_t last_update, now_update;               /* Sampling cycle count, ubit ms */
static uint8_t tx, rx;
static uint8_t tx_buff[14] = {0xff};
uint8_t mpu_buff[14];                          /* buffer to save imu raw data */
uint8_t ist_buff[6];                           /* buffer to save IST8310 raw data */
uint8_t id;

MPU_TypeDef mpu_data = {0};
MPU_ImuTypeDef mpu_imu = {0};

/**
  * @brief  fast inverse square-root, to calculate 1/Sqrt(x)
  * @param  x: the number need to be calculated
  * @retval 1/Sqrt(x)
  * @usage  call in imu_ahrs_update() function
  */
float inv_sqrt(float x) {
    float half_x = 0.5f * x;
    float y = x;
    long i = *(long *) &y;
    i = 0x5F3759DF - (i >> 1);
    y = *(float *) &i;
    y = y * (1.5f - (half_x * y * y));
    return y;
}

/**
  * @brief  write a byte of data to specified register
  * @param  reg:  the address of register to be written
  *         data: data to be written
  * @retval
  * @usage  call in ist_reg_write_by_mpu(),
  *                 ist_reg_read_by_mpu(),
  *                 mpu_master_i2c_auto_read_config(),
  *                 ist8310_init(),
  *                 mpu_set_gyro_fsr(),
  *                 mpu_set_accel_fsr(),
  *                 mpu_device_init() function
  */
uint8_t MPU_WriteByte(uint8_t const reg, uint8_t const data) {
    MPU_NSS_LOW;
    tx = reg & 0x7F;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    tx = data;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    MPU_NSS_HIGH;
    return 0;
}

/**
  * @brief  read a byte of data from specified register
  * @param  reg: the address of register to be read
  * @retval
  * @usage  call in ist_reg_read_by_mpu(),
  *                 mpu_device_init() function
  */
uint8_t MPU_ReadByte(uint8_t const reg) {
    MPU_NSS_LOW;
    tx = reg | 0x80;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    MPU_NSS_HIGH;
    return rx;
}

/**
  * @brief  read bytes of data from specified register
  * @param  reg: address from where data is to be written
  * @retval
  * @usage  call in ist8310_get_data(),
  *                 mpu_get_data(),
  *                 mpu_offset_call() function
  */
uint8_t MPU_ReadBytes(uint8_t const regAddr, uint8_t *pData, uint8_t len) {
    MPU_NSS_LOW;
    tx = regAddr | 0x80;
    tx_buff[0] = tx;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, tx_buff, pData, len, 55);
    MPU_NSS_HIGH;
    return 0;
}

/**
	* @brief  write IST8310 register through MPU6500's I2C Master
	* @param  addr: the address to be read of IST8310's register
	* @retval
  * @usage  call in ist8310_init() function
	*/
static uint8_t MPU_IstRegReadByMpu(uint8_t addr)
{
    uint8_t retval;
    MPU_WriteByte(MPU6500_I2C_SLV4_REG, addr);
    HAL_Delay(10);
    MPU_WriteByte(MPU6500_I2C_SLV4_CTRL, 0x80);
    HAL_Delay(10);
    retval = MPU_ReadByte(MPU6500_I2C_SLV4_DI);
    /* turn off slave4 after read */
    MPU_WriteByte(MPU6500_I2C_SLV4_CTRL, 0x00);
    HAL_Delay(10);
    return retval;
}

/**
    * @brief  write IST8310 register through MPU6500's I2C master
    * @param  addr: the address to be written of IST8310's register
    *         data: data to be written
	* @retval
    * @usage  call in ist8310_init() function
*/
static void MPU_IstRegWriteByMpu(uint8_t addr, uint8_t data) {
    /* turn off slave 1 at first */
    MPU_WriteByte(MPU6500_I2C_SLV1_CTRL, 0x00);
    HAL_Delay(2);
    MPU_WriteByte(MPU6500_I2C_SLV1_REG, addr);
    HAL_Delay(2);
    MPU_WriteByte(MPU6500_I2C_SLV1_DO, data);
    HAL_Delay(2);
    /* turn on slave 1 with one byte transmitting */
    MPU_WriteByte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* wait longer to ensure the data is transmitted from slave 1 */
    HAL_Delay(10);
}

/**
	* @brief    initialize the MPU6500 I2C Slave 0 for I2C reading.
    * @param    device_address: slave device address, Address[6:0]
	* @retval   void
	* @note
*/
static void MPU_MasterI2CAutoReadConfig(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num) {
    /*
     * configure the device address of the IST8310
     * use slave1, auto transmit single measure mode
     */
    MPU_WriteByte(MPU6500_I2C_SLV1_ADDR, device_address);
    HAL_Delay(2);
    MPU_WriteByte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    HAL_Delay(2);
    MPU_WriteByte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
    HAL_Delay(2);

    /* use slave0,auto read data */
    MPU_WriteByte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
    HAL_Delay(2);
    MPU_WriteByte(MPU6500_I2C_SLV0_REG, reg_base_addr);
    HAL_Delay(2);

    /* every eight mpu6500 internal samples one i2c master read */
    MPU_WriteByte(MPU6500_I2C_SLV4_CTRL, 0x03);
    HAL_Delay(2);
    /* enable slave 0 and 1 access delay */
    MPU_WriteByte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    HAL_Delay(2);
    /* enable slave 1 auto transmit */
    MPU_WriteByte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* Wait 6ms (minimum waiting time for 16 times internal average setup) */
    HAL_Delay(6);
    /* enable slave 0 with data_num bytes reading */
    MPU_WriteByte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
    HAL_Delay(2);
}

/**
    * @brief  Initializes the IST8310 device
	* @param
	* @retval
    * @usage  call in mpu_device_init() function
*/
uint8_t MPU_Ist8310Init() {
    /* enable iic master mode */
    MPU_WriteByte(MPU6500_USER_CTRL, 0x30);
    HAL_Delay(10);
    /* enable iic 400khz */
    MPU_WriteByte(MPU6500_I2C_MST_CTRL, 0x0d);
    HAL_Delay(10);

    /* turn on slave 1 for ist write and slave 4 to ist read */
    MPU_WriteByte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
    HAL_Delay(10);
    MPU_WriteByte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
    HAL_Delay(10);

    /* IST8310_R_CONFB 0x01 = device rst */
    MPU_IstRegWriteByMpu(IST8310_R_CONFB, 0x01);
    HAL_Delay(10);
    if (IST8310_DEVICE_ID_A != MPU_IstRegReadByMpu(IST8310_WHO_AM_I))
        return 1;

    /* soft reset */
    MPU_IstRegWriteByMpu(IST8310_R_CONFB, 0x01);
    HAL_Delay(10);

    /* config as ready mode to access register */
    MPU_IstRegWriteByMpu(IST8310_R_CONFA, 0x00);
    if (MPU_IstRegReadByMpu(IST8310_R_CONFA) != 0x00)
        return 2;
    HAL_Delay(10);

    /* normal state, no int */
    MPU_IstRegWriteByMpu(IST8310_R_CONFB, 0x00);
    if (MPU_IstRegReadByMpu(IST8310_R_CONFB) != 0x00)
        return 3;
    HAL_Delay(10);

    /* config low noise mode, x,y,z axis 16 time 1 avg */
    MPU_IstRegWriteByMpu(IST8310_AVGCNTL, 0x24); //100100
    if (MPU_IstRegReadByMpu(IST8310_AVGCNTL) != 0x24)
        return 4;
    HAL_Delay(10);

    /* Set/Reset pulse duration setup,normal mode */
    MPU_IstRegWriteByMpu(IST8310_PDCNTL, 0xc0);
    if (MPU_IstRegReadByMpu(IST8310_PDCNTL) != 0xc0)
        return 5;
    HAL_Delay(10);

    /* turn off slave1 & slave 4 */
    MPU_WriteByte(MPU6500_I2C_SLV1_CTRL, 0x00);
    HAL_Delay(10);
    MPU_WriteByte(MPU6500_I2C_SLV4_CTRL, 0x00);
    HAL_Delay(10);

    /* configure and turn on slave 0 */
    MPU_MasterI2CAutoReadConfig(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    HAL_Delay(100);
    return 0;
}

/**
	* @brief  get the data of IST8310
    * @param  buff: the buffer to save the data of IST8310
	* @retval
    * @usage  call in mpu_get_data() function
*/
void MPU_Ist8310GetData(uint8_t *buff) {
    MPU_ReadBytes(MPU6500_EXT_SENS_DATA_00, buff, 6);
}

/**
	* @brief  get the data of imu
    * @param
	* @retval
    * @usage  call in main() function
*/
void MPU_GetData() {
    MPU_ReadBytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

    mpu_data.ax = mpu_buff[0] << 8 | mpu_buff[1];
    mpu_data.ay = mpu_buff[2] << 8 | mpu_buff[3];
    mpu_data.az = mpu_buff[4] << 8 | mpu_buff[5];
    mpu_data.temp = mpu_buff[6] << 8 | mpu_buff[7];

    mpu_data.gx = ((mpu_buff[8] << 8 | mpu_buff[9]) - mpu_data.gx_offset);
    mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset);
    mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset);
    MPU_Ist8310GetData(ist_buff);

    memcpy(&mpu_data.mx, ist_buff, 6);

    memcpy(&mpu_imu.ax, &mpu_data.ax, 6 * sizeof(int16_t));

    mpu_imu.temp = 21 + mpu_data.temp / 333.87f;
    /* 2000dps -> rad/s */
    mpu_imu.wx = mpu_data.gx / 16.384f / 57.3f;
    mpu_imu.wy = mpu_data.gy / 16.384f / 57.3f;
    mpu_imu.wz = mpu_data.gz / 16.384f / 57.3f;
}

/**
	* @brief  set imu 6500 gyroscope measure range
    * @param  fsr: range(0,±250dps;1,±500dps;2,±1000dps;3,±2000dps)
	* @retval
    * @usage  call in mpu_device_init() function
*/
uint8_t MPU_SetGyroFsr(uint8_t fsr) {
    return MPU_WriteByte(MPU6500_GYRO_CONFIG, fsr << 3);
}

/**
	* @brief  set imu 6050/6500 accelerate measure range
    * @param  fsr: range(0,±2g;1,±4g;2,±8g;3,±16g)
	* @retval
    * @usage  call in mpu_device_init() function
*/
uint8_t MPU_SetAccelFsr(uint8_t fsr) {
    return MPU_WriteByte(MPU6500_ACCEL_CONFIG, fsr << 3);
}

/**
	* @brief  get the offset data of MPU6500
    * @param
	* @retval
    * @usage  call in main() function
*/
void MPU_OffsetCall(void) {
    int i;
    for (i = 0; i < 300; i++) {
        MPU_ReadBytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

        mpu_data.ax_offset += mpu_buff[0] << 8 | mpu_buff[1];
        mpu_data.ay_offset += mpu_buff[2] << 8 | mpu_buff[3];
        mpu_data.az_offset += mpu_buff[4] << 8 | mpu_buff[5];

        mpu_data.gx_offset += mpu_buff[8] << 8 | mpu_buff[9];
        mpu_data.gy_offset += mpu_buff[10] << 8 | mpu_buff[11];
        mpu_data.gz_offset += mpu_buff[12] << 8 | mpu_buff[13];

        HAL_Delay(5);
    }
    mpu_data.ax_offset = mpu_data.ax_offset / 300;
    mpu_data.ay_offset = mpu_data.ay_offset / 300;
    mpu_data.az_offset = mpu_data.az_offset / 300;
    mpu_data.gx_offset = mpu_data.gx_offset / 300;
    mpu_data.gy_offset = mpu_data.gy_offset / 300;
    mpu_data.gz_offset = mpu_data.gz_offset / 300;
}


/**
	* @brief  initialize imu mpu6500 and magnet meter ist3810
    * @param
	* @retval
    * @usage  call in main() function
*/
uint8_t MPU_DeviceInit(void) {
    HAL_Delay(100);

    id = MPU_ReadByte(MPU6500_WHO_AM_I);
    uint8_t i = 0;
    uint8_t MPU6500_Init_Data[10][2] = {{MPU6500_PWR_MGMT_1,     0x80},     /* Reset Device */
                                        {MPU6500_PWR_MGMT_1,     0x03},     /* Clock Source - Gyro-Z */
                                        {MPU6500_PWR_MGMT_2,     0x00},     /* Enable Acc & Gyro */
                                        {MPU6500_CONFIG,         0x04},         /* LPF 41Hz */
                                        {MPU6500_GYRO_CONFIG,    0x18},    /* +-2000dps */
                                        {MPU6500_ACCEL_CONFIG,   0x10},   /* +-8G */
                                        {MPU6500_ACCEL_CONFIG_2, 0x02}, /* enable LowPassFilter  Set Acc LPF */
                                        {MPU6500_USER_CTRL,      0x20},};    /* Enable AUX */
    for (i = 0; i < 10; i++) {
        MPU_WriteByte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
        HAL_Delay(1);
    }
    MPU_SetGyroFsr(3);
    MPU_SetAccelFsr(2);
    MPU_Ist8310Init();
    MPU_OffsetCall();
    return 0;
}

/**
	* @brief  Initialize quaternion
    * @param
	* @retval
    * @usage  call in main() function
*/
void MPU_InitQuaternion(void) {
    int16_t hx, hy;//hz;

    hx = mpu_imu.mx;
    hy = mpu_imu.my;
    //hz = imu.mz;

#ifdef BOARD_DOWN
    if (hx < 0 && hy < 0) {
        if (fabs((double) hx / hy) >= 1) {
            q0 = -0.005f;
            q1 = -0.199f;
            q2 = 0.979f;
            q3 = -0.0089f;
        } else {
            q0 = -0.008f;
            q1 = -0.555f;
            q2 = 0.83f;
            q3 = -0.002f;
        }

    } else if (hx < 0 && hy > 0) {
        if (fabs((double) hx / hy) >= 1) {
            q0 = 0.005f;
            q1 = -0.199f;
            q2 = -0.978f;
            q3 = 0.012f;
        } else {
            q0 = 0.005f;
            q1 = -0.553f;
            q2 = -0.83f;
            q3 = -0.0023f;
        }

    } else if (hx > 0 && hy > 0) {
        if (fabs((double) hx / hy) >= 1) {
            q0 = 0.0012f;
            q1 = -0.978f;
            q2 = -0.199f;
            q3 = -0.005f;
        } else {
            q0 = 0.0023f;
            q1 = -0.83f;
            q2 = -0.553f;
            q3 = 0.0023f;
        }

    } else if (hx > 0 && hy < 0) {
        if (fabs((double) hx / hy) >= 1) {
            q0 = 0.0025f;
            q1 = 0.978f;
            q2 = -0.199f;
            q3 = 0.008f;
        } else {
            q0 = 0.0025f;
            q1 = 0.83f;
            q2 = -0.56f;
            q3 = 0.0045f;
        }
    }
#else
    if (hx < 0 && hy < 0)
    {
        if (fabs(hx / hy) >= 1)
        {
            q0 = 0.195;
            q1 = -0.015;
            q2 = 0.0043;
            q3 = 0.979;
        }
        else
        {
            q0 = 0.555;
            q1 = -0.015;
            q2 = 0.006;
            q3 = 0.829;
        }

    }
    else if (hx < 0 && hy > 0)
    {
        if(fabs(hx / hy) >= 1)
        {
            q0 = -0.193;
            q1 = -0.009;
            q2 = -0.006;
            q3 = 0.979;
        }
        else
        {
            q0 = -0.552;
            q1 = -0.0048;
            q2 = -0.0115;
            q3 = 0.8313;
        }

    }
    else if (hx > 0 && hy > 0)
    {
        if(fabs(hx / hy) >= 1)
        {
            q0 = -0.9785;
            q1 = 0.008;
            q2 = -0.02;
            q3 = 0.195;
        }
        else
        {
            q0 = -0.9828;
            q1 = 0.002;
            q2 = -0.0167;
            q3 = 0.5557;
        }

    }
    else if (hx > 0 && hy < 0)
    {
        if(fabs(hx / hy) >= 1)
        {
            q0 = -0.979;
            q1 = 0.0116;
            q2 = -0.0167;
            q3 = -0.195;
        }
        else
        {
            q0 = -0.83;
            q1 = 0.014;
            q2 = -0.012;
            q3 = -0.556;
        }
    }
#endif
}

/**
	* @brief  update imu AHRS
  * @param
	* @retval
  * @usage  call in main() function
	*/
void MPU_ImuAhrsUpdate(void)
{
    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez, halfT;
    float tempq0,tempq1,tempq2,tempq3;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    gx = mpu_imu.wx;
    gy = mpu_imu.wy;
    gz = mpu_imu.wz;
    ax = mpu_imu.ax;
    ay = mpu_imu.ay;
    az = mpu_imu.az;
    mx = mpu_imu.mx;
    my = mpu_imu.my;
    mz = mpu_imu.mz;

    now_update  = HAL_GetTick(); //ms
    halfT       = ((float)(now_update - last_update) / 2000.0f);
    last_update = now_update;

    /* Fast inverse square-root */
    norm = inv_sqrt(ax*ax + ay*ay + az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

#ifdef IST8310
    norm = inv_sqrt(mx*mx + my*my + mz*mz);
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm;
#else
    mx = 0;
		my = 0;
		mz = 0;
#endif
    /* compute reference direction of flux */
    hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
    hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
    hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz;

    /* estimated direction of gravity and flux (v and w) */
    vx = 2.0f*(q1q3 - q0q2);
    vy = 2.0f*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
    wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
    wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);

    /*
     * error is sum of cross product between reference direction
     * of fields and direction measured by sensors
     */
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    /* PI */
    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
        exInt = exInt + ex * Ki * halfT;
        eyInt = eyInt + ey * Ki * halfT;
        ezInt = ezInt + ez * Ki * halfT;

        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
    }

    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
    tempq1 = q1 + (q0*gx + q2*gz - q3*gy) * halfT;
    tempq2 = q2 + (q0*gy - q1*gz + q3*gx) * halfT;
    tempq3 = q3 + (q0*gz + q1*gy - q2*gx) * halfT;

    /* normalise quaternion */
    norm = inv_sqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;
}

/**
	* @brief  update imu attitude
    * @param
	* @retval
    * @usage  call in main() function
*/
void MPU_ImuAttitudeUpdate(void)
{
    /* yaw    -pi----pi */
    mpu_imu.yaw = -atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1)* 57.3;
    /* pitch  -pi/2----pi/2 */
    mpu_imu.pit = -asin(-2*q1*q3 + 2*q0*q2)* 57.3;
    /* roll   -pi----pi  */
    mpu_imu.rol =  atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)* 57.3;
}

void MPU_Update(void)
{
    MPU_GetData();
    MPU_ImuAhrsUpdate();
    MPU_ImuAttitudeUpdate();
}
