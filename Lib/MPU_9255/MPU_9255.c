/**
  ******************************************************************************
  * File Name          : MPU9255.c
  * Description        : MPU9255 函式庫標頭檔
  ******************************************************************************
  * MIAT 實驗室製作
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "MPU_9255.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

// System constants
#define gyroMeasError 3.14159265358979f * (5.0f / 180.0f)     // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError                // compute beta
#define PI 3.14159265358979323846f

#define GyroMeasDrift PI * (1.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
    
// Pin definitions
#define intPin 12  // These can be changed, 2 and 3 are the Arduinos ext int pins



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
void MPU_9255_delay(uint32_t ms) __attribute__((weak)) ;

MPU_9255_t* MPU_9255_new(int addr, uint8_t smplrt_div, uint8_t mmode, uint8_t afs, uint8_t gfs, uint8_t mfs, MPU_9255_i2cTransiver_t reader, MPU_9255_i2cTransiver_t writer){
  MPU_9255_t* nmpu = (MPU_9255_t *)malloc(sizeof(MPU_9255_t));
  nmpu->addr = addr;
  nmpu->Smplrt_div = smplrt_div;
  nmpu->Mscale = mmode;
  nmpu->Ascale = afs;
  nmpu->Gscale = gfs;
  nmpu->Mscale = mfs;
  nmpu->writer = writer;
  nmpu->reader = reader;
  
  nmpu->q[0] = 1.0f;
  nmpu->q[1] = 0.0f;
  nmpu->q[2] = 0.0f;
  nmpu->q[3] = 0.0f;

  nmpu->eInt[0] = 0.0f;
  nmpu->SEq_1 = 1.0f;
  nmpu->SEq_2 = 0.0f;
  nmpu->SEq_3 = 0.0f;
  nmpu->SEq_4 = 0.0f;
  return nmpu;
}

void MPU_9255_destory(MPU_9255_t* nmpu){
  if(nmpu != NULL) free(nmpu);
}

void MPU_9255_SetTransiver(MPU_9255_t *hmpu, MPU_9255_i2cTransiver_t reader, MPU_9255_i2cTransiver_t writer){
  hmpu->writer = writer;
  hmpu->reader = reader;
}

void MPU_9255_delay(uint32_t ms) {
  asm("nop");
}

void writeByte(MPU_9255_t *hmpu, uint8_t address, uint8_t subAddress, uint8_t data){
  uint8_t pData[2];
  pData[0] = subAddress;
  pData[1] = data;
  hmpu->writer(address, pData, 2);
}

char readByte(MPU_9255_t *hmpu, uint8_t address, uint8_t subAddress)
{
    uint8_t data[1]; // `data` will store the register data     
    uint8_t data_write[1];
    data_write[0] = subAddress;
    hmpu->writer(address, data_write, 1); // no stop
    hmpu->reader(address, data, 1); 
    return data[0]; 
}

void readBytes(MPU_9255_t *hmpu, uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{     
    uint8_t data[14];
    uint8_t data_write[1];
    data_write[0] = subAddress;
    hmpu->writer(address, data_write, 1); // no stop
    hmpu->reader(address, data, count); 
    for(int ii = 0; ii < count; ii++) {
      dest[ii] = data[ii];
    }
} 
 

uint8_t MPU_9255_whoami(MPU_9255_t *hmpu){
  return readByte(hmpu, hmpu->addr, WHO_AM_I_MPU9255); 
}
  

double getEulerAngleSampleRate(MPU_9255_t *hmpu){
  return hmpu->sample_rate;
}


void getMres(MPU_9255_t *hmpu) {
  switch (hmpu->Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
  case MFS_14BITS:
      hmpu->mRes = 10.0*4219.0/8190.0; // Proper scale to return milliGauss
      break;
  case MFS_16BITS:
      //-32768~32767 * 10.0*4219.0/32760.0=> -42200~42200
      hmpu->mRes = 10.0*4219.0/32760.0; // Proper scale to return milliGauss
      break;
  }
}


void getGres(MPU_9255_t *hmpu) {
  switch (hmpu->Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
    hmpu->gRes = 250.0/32768.0;
    break;
    case GFS_500DPS:
    hmpu->gRes = 500.0/32768.0;
    break;
    case GFS_1000DPS:
    hmpu->gRes = 1000.0/32768.0;
    break;
    case GFS_2000DPS:
    hmpu->gRes = 2000.0/32768.0;
    break;
  }
}


void getAres(MPU_9255_t *hmpu) {
  switch (hmpu->Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
  case AFS_2G:
    hmpu->aRes = 2.0/32768.0;
    break;
  case AFS_4G:
    hmpu->aRes = 4.0/32768.0;
    break;
  case AFS_8G:
    hmpu->aRes = 8.0/32768.0;
    break;
  case AFS_16G:
    hmpu->aRes = 16.0/32768.0;
    break;
  }
}


void getRes(MPU_9255_t *hmpu){

  getAres(hmpu); // Get accelerometer sensitivity
  getGres(hmpu); // Get gyro sensitivity
  getMres(hmpu); // Get magnetometer sensitivity
  //pc.printf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f/aRes);
  //pc.printf("Gyroscope sensitivity is %f LSB/deg/s \n\r", 1.0f/gRes);
  //pc.printf("Magnetometer sensitivity is %f LSB/G \n\r", 1.0f/mRes);
  hmpu->magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
  hmpu->magbias[1] = +120.;  // User environmental y-axis correction in milliGauss
  hmpu->magbias[2] = +125.;  // User environmental z-axis correction in milliGauss
  
}

void MPU_9255_readAccelData(MPU_9255_t *hmpu, double *ax, double *ay, double *az)
{
  int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(hmpu, hmpu->addr, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  accelCount[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  accelCount[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  accelCount[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
  // Now we'll calculate the accleration value into actual g's
  *ax = (double)accelCount[0]* hmpu->aRes - hmpu->accelBias[0];  // get actual g value, this depends on scale being set
  *ay = (double)accelCount[1]* hmpu->aRes - hmpu->accelBias[1];  
  *az = (double)accelCount[2]* hmpu->aRes - hmpu->accelBias[2];  
      
}

void MPU_9255_readGyroData(MPU_9255_t *hmpu, double *gx, double *gy, double *gz)
{
  int16_t gyroCount[3]; // Stores the 16-bit signed gyro sensor output
  uint8_t rawData[6];   // x/y/z gyro register data stored here
  readBytes(hmpu,hmpu->addr, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  gyroCount[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gyroCount[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gyroCount[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  // Calculate the gyro value into actual degrees per second
  *gx = (double)gyroCount[0]* hmpu->gRes - hmpu->gyroBias[0];  // get actual gyro value, this depends on scale being set
  *gy = (double)gyroCount[1]* hmpu->gRes - hmpu->gyroBias[1];  
  *gz = (double)gyroCount[2]* hmpu->gRes - hmpu->gyroBias[2];   
}

void MPU_9255_readMagData(MPU_9255_t *hmpu, double *mx, double *my, double *mz)
{
  int16_t magCount[3]; // Stores the 16-bit signed magnetometer sensor output
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if(readByte(hmpu, AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
    readBytes(hmpu, AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
    uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
      magCount[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
      magCount[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) ;  // Data stored as little Endian
      magCount[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) ; 
      // Calculate the magnetometer values in milliGauss
      // Include factory calibration per data sheet and user environmental corrections
      //mRes = -42200~42200 for 16 bit ADC
      //magCalibration = 0.5 ~ 1.5
      //magCount = -32768~32767
      *mx = (double)magCount[0]*hmpu->mRes*hmpu->magCalibration[0] - hmpu->magbias[0];  // get actual magnetometer value, this depends on scale being set
      *my = (double)magCount[1]*hmpu->mRes*hmpu->magCalibration[1] - hmpu->magbias[1];  
      *mz = (double)magCount[2]*hmpu->mRes*hmpu->magCalibration[2] - hmpu->magbias[2];   
    }
  }
}

double MPU_9255_readTempData(MPU_9255_t *hmpu)
{
  int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(hmpu, hmpu->addr, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
  tempCount = (int16_t)(((int16_t)rawData[0]) << 8 | rawData[1]); // Turn the MSB and LSB into a 16-bit value
  return ((double) tempCount) / 333.87f + 21.0f;  // Temperature in degrees Centigrade
}

double MPU_9255_calcMagForce(double mx, double my, double mz){
  return pow(mx * mx + my * my + mz * mz, 0.5);
}

double MPU_9255_calcMagHorizontal(double mx, double my){
  return pow(mx * mx + my * my, 0.5);
}

double MPU_9255_calcMagInclination(double mx, double my, double mz){
  return atan( mz / MPU_9255_calcMagHorizontal(mx,my) );
}

double MPU_9255_calcMagDeclination(double mx, double my){
  return atan( my / mx );
}

void resetMPU9255(MPU_9255_t *hmpu ) {
  // reset device
  writeByte(hmpu,hmpu->addr,PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  MPU_9255_delay(0.1);
}
  
void initAK8963(MPU_9255_t *hmpu, double * destination){
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(hmpu,AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  MPU_9255_delay(0.01);
  writeByte(hmpu,AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  MPU_9255_delay(0.01);
  readBytes(hmpu,AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  
  //-0.5~0.5 + 1.0 => 0.5 ~ 1.5
  destination[0] =  (double)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (double)(rawData[1] - 128)/256.0f + 1.0f;  
  destination[2] =  (double)(rawData[2] - 128)/256.0f + 1.0f; 
  writeByte(hmpu,AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  MPU_9255_delay(0.01);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(hmpu,AK8963_ADDRESS, AK8963_CNTL, hmpu->Mscale << 4 | hmpu->Mmode); // Set magnetometer data resolution and sample ODR
  MPU_9255_delay(0.01);
}


void initMPU9255(MPU_9255_t *hmpu ){  
  // Initialize MPU9255 device
  // wake up device
  writeByte(hmpu,hmpu->addr, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  MPU_9255_delay(0.1); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

  // get stable time source
  writeByte(hmpu,hmpu->addr, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

  // Configure Gyro and Accelerometer
  // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
  // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
  // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
  writeByte(hmpu,hmpu->addr, CONFIG, 0x03);  

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(hmpu,hmpu->addr, SMPLRT_DIV, hmpu->Smplrt_div);  // Use a 200 Hz rate; the same rate set in CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c =  readByte(hmpu,hmpu->addr, GYRO_CONFIG);
  writeByte(hmpu,hmpu->addr, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  writeByte(hmpu,hmpu->addr, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(hmpu,hmpu->addr, GYRO_CONFIG, c | hmpu->Gscale << 3); // Set full scale range for the gyro

  // Set accelerometer configuration
  c =  readByte(hmpu,hmpu->addr, ACCEL_CONFIG);
  writeByte(hmpu,hmpu->addr, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  writeByte(hmpu,hmpu->addr, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(hmpu,hmpu->addr, ACCEL_CONFIG, c | hmpu->Ascale << 3); // Set full scale range for the accelerometer 

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(hmpu,hmpu->addr, ACCEL_CONFIG2);
  writeByte(hmpu,hmpu->addr, ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  writeByte(hmpu,hmpu->addr, ACCEL_CONFIG2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
  writeByte(hmpu,hmpu->addr, INT_PIN_CFG, 0x22);    
  writeByte(hmpu,hmpu->addr, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9255(MPU_9255_t *hmpu, double * dest1, double * dest2)
{  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device, reset all registers, clear gyro and accelerometer bias registers
  writeByte(hmpu,hmpu->addr, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  MPU_9255_delay(0.1);  

  // get stable time source
  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeByte(hmpu,hmpu->addr, PWR_MGMT_1, 0x01);  
  writeByte(hmpu,hmpu->addr, PWR_MGMT_2, 0x00); 
  MPU_9255_delay(0.2);

  // Configure device for bias calculation
  writeByte(hmpu,hmpu->addr, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(hmpu,hmpu->addr, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(hmpu,hmpu->addr, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(hmpu,hmpu->addr, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(hmpu,hmpu->addr, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(hmpu,hmpu->addr, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  MPU_9255_delay(0.015);

  // Configure MPU9255 gyro and accelerometer for bias calculation
  writeByte(hmpu,hmpu->addr, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(hmpu,hmpu->addr, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(hmpu,hmpu->addr, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(hmpu,hmpu->addr, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(hmpu,hmpu->addr, USER_CTRL, 0x40);   // Enable FIFO  
  writeByte(hmpu,hmpu->addr, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9255)
  MPU_9255_delay(0.04); // accumulate 40 samples in 80 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(hmpu,hmpu->addr, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(hmpu,hmpu->addr, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(hmpu,hmpu->addr, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
  }
  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

/// Push gyro biases to hardware registers
  writeByte(hmpu,hmpu->addr, XG_OFFSET_H, data[0]);
  writeByte(hmpu,hmpu->addr, XG_OFFSET_L, data[1]);
  writeByte(hmpu,hmpu->addr, YG_OFFSET_H, data[2]);
  writeByte(hmpu,hmpu->addr, YG_OFFSET_L, data[3]);
  writeByte(hmpu,hmpu->addr, ZG_OFFSET_H, data[4]);
  writeByte(hmpu,hmpu->addr, ZG_OFFSET_L, data[5]);

  dest1[0] = (double) gyro_bias[0]/(double) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (double) gyro_bias[1]/(double) gyrosensitivity;
  dest1[2] = (double) gyro_bias[2]/(double) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(hmpu,hmpu->addr, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(hmpu,hmpu->addr, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(hmpu,hmpu->addr, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for(ii = 0; ii < 3; ii++) {
  if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9255
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
  
  writeByte(hmpu,hmpu->addr, XA_OFFSET_H, data[0]);
  writeByte(hmpu,hmpu->addr, XA_OFFSET_L, data[1]);
  writeByte(hmpu,hmpu->addr, YA_OFFSET_H, data[2]);
  writeByte(hmpu,hmpu->addr, YA_OFFSET_L, data[3]);
  writeByte(hmpu,hmpu->addr, ZA_OFFSET_H, data[4]);
  writeByte(hmpu,hmpu->addr, ZA_OFFSET_L, data[5]);

// Output scaled accelerometer biases for manual subtraction in the main program
  dest2[0] = (double)accel_bias[0]/(double)accelsensitivity; 
  dest2[1] = (double)accel_bias[1]/(double)accelsensitivity;
  dest2[2] = (double)accel_bias[2]/(double)accelsensitivity;
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9255SelfTest(MPU_9255_t *hmpu, double * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
  double factoryTrim[6];
  uint8_t FS = 0;

  writeByte(hmpu,hmpu->addr, SMPLRT_DIV, 0x00); // Set gyro sample rate to 1 kHz �����W�v
  writeByte(hmpu,hmpu->addr, CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz �C�W�o�i���I���W�v
  writeByte(hmpu,hmpu->addr, GYRO_CONFIG, 1<<FS); // Set full scale range for the gyro to 250 dps 
  writeByte(hmpu,hmpu->addr, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(hmpu,hmpu->addr, ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

  //讀兩百筆目前感測器值取平均
  for( int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer
    //讀加速度值
    readBytes(hmpu,hmpu->addr, ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
    //讀陀螺儀值
    readBytes(hmpu,hmpu->addr, GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }
  
  for (int ii =0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }
  
  // Configure the accelerometer for self-test
  writeByte(hmpu,hmpu->addr, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeByte(hmpu,hmpu->addr, GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  MPU_9255_delay(0.25); // Delay a while to let the device stabilize

  for( int ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer

    readBytes(hmpu,hmpu->addr, ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(hmpu,hmpu->addr, GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }
  
  for (int ii =0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }
  
  //切回一般感測資料模式
  // Configure the gyro and accelerometer for normal operation
  writeByte(hmpu,hmpu->addr, ACCEL_CONFIG, 0x00);
  writeByte(hmpu,hmpu->addr, GYRO_CONFIG, 0x00);
  MPU_9255_delay(0.25); // Delay a while to let the device stabilize

  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  selfTest[0] = readByte(hmpu,hmpu->addr, SELF_TEST_X_ACCEL); // X-axis accel self-test results
  selfTest[1] = readByte(hmpu,hmpu->addr, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
  selfTest[2] = readByte(hmpu,hmpu->addr, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
  selfTest[3] = readByte(hmpu,hmpu->addr, SELF_TEST_X_GYRO); // X-axis gyro self-test results
  selfTest[4] = readByte(hmpu,hmpu->addr, SELF_TEST_Y_GYRO); // Y-axis gyro self-test results
  selfTest[5] = readByte(hmpu,hmpu->addr, SELF_TEST_Z_GYRO); // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
  factoryTrim[0] = (double)(2620/1<<FS)*(pow( 1.01f , ((double)selfTest[0] - 1.0f) )); // FT[Xa] factory trim calculation
  factoryTrim[1] = (double)(2620/1<<FS)*(pow( 1.01f , ((double)selfTest[1] - 1.0f) )); // FT[Ya] factory trim calculation
  factoryTrim[2] = (double)(2620/1<<FS)*(pow( 1.01f , ((double)selfTest[2] - 1.0f) )); // FT[Za] factory trim calculation
  factoryTrim[3] = (double)(2620/1<<FS)*(pow( 1.01f , ((double)selfTest[3] - 1.0f) )); // FT[Xg] factory trim calculation
  factoryTrim[4] = (double)(2620/1<<FS)*(pow( 1.01f , ((double)selfTest[4] - 1.0f) )); // FT[Yg] factory trim calculation
  factoryTrim[5] = (double)(2620/1<<FS)*(pow( 1.01f , ((double)selfTest[5] - 1.0f) )); // FT[Zg] factory trim calculation

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++) {
    destination[i] = 100.0f*((double)(aSTAvg[i] - aAvg[i]))/factoryTrim[i]; // Report percent differences
    destination[i+3] = 100.0f*((double)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
  }
   
}

void MPU_9255_initSensor(MPU_9255_t *hmpu){
  
    //software reset by setting register
    resetMPU9255(hmpu); // Reset registers to default in preparation for device calibration
    //self test
    MPU9255SelfTest(hmpu, hmpu->SelfTest); // Start by performing self test and reporting values
    calibrateMPU9255(hmpu, hmpu->gyroBias, hmpu->accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
    MPU_9255_delay(2);
    
    initMPU9255(hmpu); 
    initAK8963(hmpu, hmpu->magCalibration);
    MPU_9255_delay(1);
  
    //get resolution of MPU9255
    getRes(hmpu);
}

void MPU_9255_printInfo(MPU_9255_t *hmpu){
    printf("\n\r--------------- start of Sensor Infomation ---------------\n\r");
    printf("x-axis self test: acceleration trim within : %f %% of factory value\n\r", hmpu->SelfTest[0]);  
    printf("y-axis self test: acceleration trim within : %f %% of factory value\n\r", hmpu->SelfTest[1]);  
    printf("z-axis self test: acceleration trim within : %f %% of factory value\n\r", hmpu->SelfTest[2]);  
    printf("x-axis self test: gyration trim within : %f %% of factory value\n\r", hmpu->SelfTest[3]);  
    printf("y-axis self test: gyration trim within : %f %% of factory value\n\r", hmpu->SelfTest[4]);  
    printf("z-axis self test: gyration trim within : %f %% of factory value\n\r", hmpu->SelfTest[5]); 
  
    printf("x gyro bias = %f\n\r", hmpu->gyroBias[0]);
    printf("y gyro bias = %f\n\r", hmpu->gyroBias[1]);
    printf("z gyro bias = %f\n\r", hmpu->gyroBias[2]);
    printf("x accel bias = %f\n\r", hmpu->accelBias[0]);
    printf("y accel bias = %f\n\r", hmpu->accelBias[1]);
    printf("z accel bias = %f\n\r", hmpu->accelBias[2]);
  
    printf("MPU9255 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
  
    printf("AK8963 initialized for active data mode....\n\r"); // Initialize device for active mode read of magnetometer
    printf("Accelerometer full-scale range = %f  g\n\r", 2.0f*(double)(1<<hmpu->Ascale));
    printf("Gyroscope full-scale range = %f  deg/s\n\r", 250.0f*(double)(1<<hmpu->Gscale));
    if(hmpu->Mscale == 0) printf("Magnetometer resolution = 14  bits\n\r");
    else if(hmpu->Mscale == 1) printf("Magnetometer resolution = 16  bits\n\r");
    if(hmpu->Mmode == 2) printf("Magnetometer ODR = 8 Hz\n\r");
    else if(hmpu->Mmode == 6) printf("Magnetometer ODR = 100 Hz\n\r");
    printf("Accelerometer and Gyroscope sampling rate = %d Hz\n\r", 1000 / (1 + hmpu->Smplrt_div));
    
    printf("--------------- end of Sensor Infomation ---------------\n\r");
}

uint8_t MPU_9255_isDataReady(MPU_9255_t *hmpu){
  return readByte(hmpu,hmpu->addr, INT_STATUS) & 0x01;
}


// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration and rotation rate to produce a quaternion-based estimate of relative
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MPU_9255_filterUpdate(MPU_9255_t *hmpu, double gx, double gy, double gz, double ax, double ay, double az, double deltat)
{
  // Local system variables
  double norm;                                                            // vector norm
  double SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4;  // quaternion derrivative from gyroscopes elements
  double f_1, f_2, f_3;                                                   // objective function elements
  double J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;              // objective function Jacobian elements
  double SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;              // estimated direction of the gyroscope error
  // Axulirary variables to avoid reapeated calcualtions
  double halfSEq_1 = 0.5f * hmpu->SEq_1;
  double halfSEq_2 = 0.5f * hmpu->SEq_2;
  double halfSEq_3 = 0.5f * hmpu->SEq_3;
  double halfSEq_4 = 0.5f * hmpu->SEq_4;
  double twoSEq_1 = 2.0f * hmpu->SEq_1;
  double twoSEq_2 = 2.0f * hmpu->SEq_2;
  double twoSEq_3 = 2.0f * hmpu->SEq_3;
  
  // Normalise the accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  ax /= norm;
  ay /= norm;
  az /= norm;
  // Compute the objective function and Jacobian
  f_1 = twoSEq_2 * hmpu->SEq_4 - twoSEq_1 * hmpu->SEq_3 - ax;
  f_2 = twoSEq_1 * hmpu->SEq_2 + twoSEq_3 * hmpu->SEq_4 - ay;
  f_3 = 1.0f - twoSEq_2 * hmpu->SEq_2 - twoSEq_3 * hmpu->SEq_3 - az;
  J_11or24 = twoSEq_3;                                                    // J_11 negated in matrix multiplication
  J_12or23 = 2.0f * hmpu->SEq_4;
  J_13or22 = twoSEq_1;                                                    // J_12 negated in matrix multiplication
  J_14or21 = twoSEq_2;
  J_32 = 2.0f * J_14or21;                                                 // negated in matrix multiplication
  J_33 = 2.0f * J_11or24;                                                 // negated in matrix multiplication
  // Compute the gradient (matrix multiplication)
  SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
  SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
  SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
  SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
  // Normalise the gradient
  norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
  SEqHatDot_1 /= norm;
  SEqHatDot_2 /= norm;
  SEqHatDot_3 /= norm;
  SEqHatDot_4 /= norm;
  // Compute the quaternion derrivative measured by gyroscopes
  SEqDot_omega_1 = -halfSEq_2 * gx - halfSEq_3 * gy - halfSEq_4 * gz;
  SEqDot_omega_2 = halfSEq_1 * gx + halfSEq_3 * gz - halfSEq_4 * gy;
  SEqDot_omega_3 = halfSEq_1 * gy - halfSEq_2 * gz + halfSEq_4 * gx;
  SEqDot_omega_4 = halfSEq_1 * gz + halfSEq_2 * gy - halfSEq_3 * gx;
  // Compute then integrate the estimated quaternion derrivative
  hmpu->SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
  hmpu->SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
  hmpu->SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
  hmpu->SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
  // Normalise quaternion
  norm = sqrt(hmpu->SEq_1 * hmpu->SEq_1 + hmpu->SEq_2 * hmpu->SEq_2 + hmpu->SEq_3 * hmpu->SEq_3 + hmpu->SEq_4 * hmpu->SEq_4);
  hmpu->SEq_1 /= norm;
  hmpu->SEq_2 /= norm;
  hmpu->SEq_3 /= norm;
  hmpu->SEq_4 /= norm;
}
void MPU_9255_getEulerDegreeFilter(MPU_9255_t *hmpu, double *yaw, double *pitch, double *roll){
  
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
  *yaw   = atan2(2.0f * (hmpu->SEq_2 * hmpu->SEq_3 + hmpu->SEq_1 * hmpu->SEq_4), hmpu->SEq_1 * hmpu->SEq_1 + hmpu->SEq_2 * hmpu->SEq_2 - hmpu->SEq_3 * hmpu->SEq_3 - hmpu->SEq_4 * hmpu->SEq_4);   
  *pitch = -asin(2.0f * (hmpu->SEq_2 * hmpu->SEq_4 - hmpu->SEq_1 * hmpu->SEq_3));
  *roll  = atan2(2.0f * (hmpu->SEq_1 * hmpu->SEq_2 + hmpu->SEq_3 * hmpu->SEq_4), hmpu->SEq_1 * hmpu->SEq_1 - hmpu->SEq_2 * hmpu->SEq_2 - hmpu->SEq_3 * hmpu->SEq_3 + hmpu->SEq_4 * hmpu->SEq_4);
  *pitch *= 180.0f / PI;
  *yaw   *= 180.0f / PI; 
  *yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
  *roll  *= 180.0f / PI;
      
}



/******************************************************************************/
/*                                                                            */ 
/******************************************************************************/

/*********************************END OF FILE**********************************/
