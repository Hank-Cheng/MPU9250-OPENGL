/*
使用センサー：MPU9250 AHRS
プログラミング内容：
九軸センサの加速度情報，角速度情報，磁気をつかい
rotation matrix で，姿勢を算出する．
*/

#include <SPI.h>
#include <Wire.h>


//if 1 print calibation infomatio
#define calibationinfo 0
   
//データの送信速度
#define rateofdata 50
//データの通信速度//serial console speeds
#define serialbaud 250000 

//Filter Parameter
#define Kp 5.0f * 5.0f    //2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f          //0.0f


//Magnetometer Registers // 磁気センサーのアドレス表
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00                  
#define SELF_TEST_Y_GYRO 0x01                                                                          
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E   
#define WOM_THR          0x1F   

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

// Using the MSENSR-9250 breakout board, ADO is set to 0 
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS     // 0.15 mG per LSB
};

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x02;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
  
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float   magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float   gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer mg
float   SelfTest[6];    // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float   GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float   GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)

float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value



uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0, sumCount = 0; // used to control display output rate
float pitch, yaw, roll;
float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

//--------------------------------------------------------------------------------------------

void setup()
{
  Wire.begin();
  Serial.begin(serialbaud);
  Serial.print("Setup  Calibationinfo...");
  
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  if(calibationinfo==1){
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
  }
  delay(500);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {  
    if(calibationinfo==1){
    Serial.println("MPU9250 is online...");
    }//calibationinfo==1
    MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
    
    if(calibationinfo==1){
    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");     Serial.print(SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");     Serial.print(SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");     Serial.print(SelfTest[5],1); Serial.println("% of factory value");
    }//calibationinfo==1
    
    calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
 
    initMPU9250(); 
    
    if(calibationinfo==1){
    Serial.println("MPU9250 initialized for active data mode...."); 
    }//calibationinfo==1
    
    // Initialize device for active mode read of acclerometer, gyroscope, and temperature
  
    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    byte d = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
    
    if(calibationinfo==1){
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
    }//calibationinfo==1
    
    // Get magnetometer calibration from AK8963 ROM
    initAK8963(magCalibration); 
    if(calibationinfo==1){
    Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
    Serial.println("Calibration values: ");
    Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
    Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
    Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
    }//calibationinfo==1
  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }

  Serial.print("START");
}

//--------------------------------------------------------------------------------------------
void loop()
{  
        //
        readAccelData(accelCount);  // Read the x/y/z adc values
        getAres();
        ax = (float)accelCount[0]*aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
        ay = (float)accelCount[1]*aRes; // - accelBias[1];   
        az = (float)accelCount[2]*aRes; // - accelBias[2];  
       
        readGyroData(gyroCount);  // Read the x/y/z adc values
        getGres();
        gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
        gy = (float)gyroCount[1]*gRes;  
        gz = (float)gyroCount[2]*gRes;   
      
        readMagData(magCount);  // Read the x/y/z adc values
        getMres();
        
        //環境の磁気調整パラメータ
        magbias[0] = 275.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
        magbias[1] = -100.;  // User environmental x-axis correction in milliGauss
        magbias[2] = +25.;  // User environmental x-axis correction in milliGauss

        // Calculate the magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental corrections
        mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
        my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];  
        mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];   

      
      Now = micros();
      deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
      lastUpdate = Now;
    
      sum += deltat; // sum for averaging filter update rate
      sumCount++;

      //四元数計算
      MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);
      //MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);

      // Serial print and/or display at 0.5 s rate independent of data rates
      delt_t = millis() - count;
      
      if (delt_t > rateofdata) { // update LCD once per half-second independent of read rate
        
       /*
        Serial.print("ax = "); Serial.print((int)1000*ax);  
        Serial.print(" ay = "); Serial.print((int)1000*ay); 
        Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
        Serial.print("gx = "); Serial.print( gx, 2); 
        Serial.print(" gy = "); Serial.print( gy, 2); 
        Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
           */
           int app =0;
        if(app==1){   
        Serial.print("mx = "); Serial.print( (int)mx ); 
        Serial.print(" my = "); Serial.print( (int)my ); 
        Serial.print(" mz = "); Serial.print( (int)mz ); Serial.print(" mG");
         Serial.print("/t"); 
     
        Serial.print(" q0 = "); Serial.print(q[0]);
        Serial.print(" qx = "); Serial.print(q[1]); 
        Serial.print(" qy = "); Serial.print(q[2]); 
        Serial.print(" qz = "); Serial.print(q[3]); 
  
        Serial.print(" RotationAngle = "); Serial.print(2*acos(q[0])*180/PI); 
        }
        /*
        yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
        pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
        roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
        */
    

  
        //roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3] ),   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]/*( 1 - 2*( q[1] * q[1] + q[2] * q[2] ))*/  );
        // pitch = asin(t2);                               ;
        yaw   = -2*acos(q[0]);//atan2(2.0f * (q[1] * q[2] - q[0] * q[3] ),( q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3] ) );   
        // roll  = (tan(q[2]/q[3]));
        //pitch =( tan(q[1]/q[3]));

        double ysqr = q[2] * q[2];
        double t0 = -2.0f * (ysqr + q[3] * q[3]) + 1.0f;
        double t1 = +2.0f * (q[1] * q[2] - q[2] * q[3]);
        double t2 = -2.0f * (q[1] * q[3] + q[0] * q[2]);
        double t3 = +2.0f * (q[2] * q[3] - q[0] * q[1]);
        double t4 = -2.0f * (q[1] * q[1] + ysqr) + 1.0f;
        
        t2 = t2 > 1.0f ? 1.0f : t2;
        t2 = t2 < -1.0f ? -1.0f : t2;

        pitch = asin(t2);
        roll = atan2(t3, t4);
        //yaw = -atan2(t1, t0);

        
        roll  *= 180.0f / PI;
        pitch *= 180.0f / PI;
        yaw   *= 180.0f / PI; 
        yaw   += 7.48;//Tsukuba Ibaraki on 2016-10-19
        
   if(app==1){
        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(round(yaw));
        Serial.print(",");
        Serial.print(round(pitch));
        Serial.print(",");
        Serial.println(round(roll));
   }


        //change the data form and sent the data to OpenGL 
        sent2openGL(yaw,pitch,roll);
        
       //Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
       /*  Serial.print("!");
          Serial.write(highByte(int(round(yaw))));
          Serial.write(lowByte(int(round(yaw))));
          Serial.write(highByte(int(round(pitch))));
          Serial.write(lowByte(int(round(pitch))));
          Serial.write(highByte(int(round(roll))));
          Serial.write(lowByte(int(round(roll))));
          
        */
        
        count = millis(); 
        sumCount = 0;
        sum = 0;    
        }
        
}//loop


//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================
void getMres() {
  switch (Mscale)
  {
  // Possible magnetometer scales (and their register bit settings) are:
  // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mRes = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
  }
}

void getGres() {
  switch (Gscale)
  {
  // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void getAres() {
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
  readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
  uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
   }
  }
}
