#ifndef __IMU_H__
  #define __IMU_H__
  
  #include <Wire.h>
  #include "Arduino.h"
  
  /* Device address */
  #define I2C_MPU6050_ADDR      0x68

  /* ------------------------------------------------------------------------ */
  /* ------------------------- REGISTERS & OP MODES ------------------------- */
  /* ------------------------------------------------------------------------ */
  
  /* Operation modes */
  #define MPU6050_FULL_INT      0           /* Accelerometer & Gyroscope & Temperature & Interrupt    */
  #define MPU6050_FULL          1           /* Accelerometer & Gyroscope & Temperature & NO Interrupt */
  #define MPU6050_ACC_INT       2           /* Accelerometer only & Interrupt                         */
  #define MPU6050_ACC_ONLY      3           /* Accelerometer only & NO Interrupt                      */
    
  /* MPU6050 Registers [consult the datasheet] */
  #define SMPRT_DIV             0x19        /* Sample rate divider of the clock (MAX ACCEL = 1KHz)    */
  #define GYRO_CONFIG           0x1B        /* Gyroscope config register, set the full scale here     */
  #define ACCEL_CONFIG          0x1C        /* Accelerometer config register, set the full scale here */
  #define INT_ENABLE            0x38        /* Enable interrupt generation, interested in data ready! */
  #define ACCEL_XOUT_H          0x3B        /* Acceleromter data, 16-bit 2's complement value(X,Y,Z)  */ 
  #define ACCEL_XOUT_L          0x3C
  #define ACCEL_YOUT_H          0x3D
  #define ACCEL_YOUT_L          0x3E
  #define ACCEL_ZOUT_H          0x3F
  #define ACCEL_ZOUT_L          0x40
  #define TEMP_OUT_H            0x41        /* Temperature data, 16-bit signed, T°C = Temp/340+36.53  */
  #define TEMP_OUT_L            0x42
  #define GYRO_XOUT_H           0x43        /* Gyroscope data, 16-bit 2's complement value (X, Y, Z)  */
  #define GYRO_XOUT_L           0x44
  #define GYRO_YOUT_H           0x45
  #define GYRO_YOUT_L           0x46
  #define GYRO_ZOUT_H           0x47
  #define GYRO_ZOUT_L           0x48
  #define PWR_MGMT_1            0x6B        /* Power management register(sleep,disable sensors,cycle) */
  #define PWR_MGMT_2            0x6C
  #define WHO_AM_I              0x75        /* I²C address, 7-bit, add[0]=AD0 pin, default add = 0x68 */

  /* I²C configs in the ESP32 */
  #define I2C_PORT_NUM          0           /* The number of I²C ports is chip specific               */
  #define I2C_FREQ_HZ           400000
  #define I2C_TIMEOUT_MS        1000

  /* ------------------------------------------------------------------------ */
  /* ---------------------------- DATA CONTAINER ---------------------------- */
  /* ------------------------------------------------------------------------ */
  
  /* Data container */
  typedef struct{  
    TwoWire * i2c_handle_mpu6050;           /* Handle to the I²C port on which the imu is mounted     */
    uint8_t addr;                           /* Slave address, 0x68 in our case                        */
    float acc_xyz[3];
    float temp;
    float gyr_xyz[3];
  }IMU;

  /* ------------------------------------------------------------------------ */
  /* ------------------------- FUNCTIONS DEFINITION ------------------------- */
  /* ------------------------------------------------------------------------ */
  
  void i2c_init_mpu6050(IMU *imu, TwoWire *i2c_port); 
  void i2c_setup_mpu6050(IMU *imu, uint8_t mode_); 
  void i2c_read_acc_mpu6050(IMU *imu); 
  void i2c_read_gyr_mpu6050(IMU *imu);
  void i2c_read_temp_mpu6050(IMU *imu);
  void i2c_write_byte_mpu6050(IMU *imu, uint8_t reg_addr, uint8_t data_);
  void i2c_write_read_mpu6050(IMU *imu, uint8_t reg_addr, uint8_t *data_, size_t how_many);
  void i2c_sleep_mpu6050(IMU *imu);

  /* ------------------------------------------------------------------------ */
  /* ------------------------ FUNCTIONS DECLARATIONS ------------------------ */
  /* ------------------------------------------------------------------------ */

  // IMU creation/Init function
  void i2c_init_mpu6050(IMU *imu, TwoWire *i2c_port){
    imu->addr = I2C_MPU6050_ADDR;
    imu->i2c_handle_mpu6050 = i2c_port;
    imu->i2c_handle_mpu6050->begin();
  }

  // Function to write a byte to the IMU
  void i2c_write_byte_mpu6050(IMU *imu, uint8_t reg_addr, uint8_t data_){
    imu->i2c_handle_mpu6050->beginTransmission(imu->addr);
    imu->i2c_handle_mpu6050->write(reg_addr);
    imu->i2c_handle_mpu6050->write(data_);
    imu->i2c_handle_mpu6050->endTransmission();
  }

  // Function to write a byte and receive a response
  void i2c_write_read_mpu6050(IMU *imu, uint8_t reg_addr, uint8_t *data_, size_t how_many){
    imu->i2c_handle_mpu6050->beginTransmission(imu->addr);
    imu->i2c_handle_mpu6050->write(reg_addr);
    imu->i2c_handle_mpu6050->endTransmission();
    
    uint16_t start_reading = millis();
    imu->i2c_handle_mpu6050->requestFrom(imu->addr, how_many);
    
    // Wait until all the bytes are received with a timeout                              
    while(imu->i2c_handle_mpu6050->available() < how_many 
          && (millis() - start_reading < I2C_TIMEOUT_MS)
         );
  
    for(uint8_t i = 0; i < how_many; i++) data_[i] = imu->i2c_handle_mpu6050->read();
  }
  
  /* Registers manipulations for settings */
  
  void i2c_setup_mpu6050(IMU *imu, uint8_t mode_){
    
    switch(mode_){
      
      case MPU6050_FULL_INT:  /* Configure the interrupt and fall into full config */
        i2c_write_byte_mpu6050(imu, INT_ENABLE, 0x01); // Data ready interrupt enabled
        
      case MPU6050_FULL:      /* Do a full mode config */
        i2c_write_byte_mpu6050(imu, PWR_MGMT_1, 0x01); // Clock ref source to Gyro's X-axis
        i2c_write_byte_mpu6050(imu, GYRO_CONFIG, 0x00); // No Self-test, full scale +- 250 °/s
        i2c_write_byte_mpu6050(imu, ACCEL_CONFIG, 0x00); // No Self-test, full scale +- 2 g
        break;
  
      case MPU6050_ACC_INT:   /* Configure the interrupt and fall into Acc-only config */
        i2c_write_byte_mpu6050(imu, INT_ENABLE, 0x01); // Data ready interrupt enabled
        
      case MPU6050_ACC_ONLY:  /* Do an Acc only config */
        i2c_write_byte_mpu6050(imu, PWR_MGMT_1, 0x28); // Disable sleep, enable cycle, disable temp sensor, set internal clock as source
        i2c_write_byte_mpu6050(imu, PWR_MGMT_2, 0xC7); // Cycle = 40Hz, and all gyro axis are put to standby
        i2c_write_byte_mpu6050(imu, ACCEL_CONFIG, 0x00); // No Self-test, full scale +- 2 g
        break;
  
      default: break;
    }
  }
  
  void i2c_sleep_mpu6050(IMU *imu){
    i2c_write_byte_mpu6050(imu, PWR_MGMT_1, 0x40); // Enable sleep
  }
  
  /* Reading data functions */
  
  void i2c_read_acc_mpu6050(IMU *imu){
    uint8_t rx_buf[6];
    i2c_write_read_mpu6050(imu, ACCEL_XOUT_H, rx_buf, sizeof(rx_buf));
    imu->acc_xyz[0] = (uint16_t)((rx_buf[0] << 8) | rx_buf[1]);
    imu->acc_xyz[1] = (uint16_t)((rx_buf[2] << 8) | rx_buf[3]);
    imu->acc_xyz[2] = (uint16_t)((rx_buf[4] << 8) | rx_buf[5]);
  }
  
  void i2c_read_gyr_mpu6050(IMU *imu){
    uint8_t rx_buf[6];
    i2c_write_read_mpu6050(imu, GYRO_XOUT_H, rx_buf, sizeof(rx_buf));
    imu->gyr_xyz[0] = (uint16_t)((rx_buf[0] << 8) | rx_buf[1]);
    imu->gyr_xyz[1] = (uint16_t)((rx_buf[2] << 8) | rx_buf[3]);
    imu->gyr_xyz[2] = (uint16_t)((rx_buf[4] << 8) | rx_buf[5]);
  }
  
  void i2c_read_temp_mpu6050(IMU *imu){
    uint8_t rx_buf[2];
    i2c_write_read_mpu6050(imu, TEMP_OUT_H, rx_buf, sizeof(rx_buf));
    float t = (float)((rx_buf[0] << 8) | rx_buf[1]);
    t = (t/340) + 36.53;
    imu->temp = t;
  }

#endif // __IMU_H__
