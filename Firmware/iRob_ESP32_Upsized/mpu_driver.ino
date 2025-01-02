// MPU-9250 driver (only Gyro and Accel)

#define MPU_ADDR          0x68

#define REG_PWRMGMT_1     0x6B
#define REG_PWRMGMT_2     0x6C
#define REG_SMPRT_DIV     0x19
#define REG_DLPFCONF      0x1A
#define REG_GYROCONF      0x1B
#define REG_ACCLCONF      0x1C
#define REG_ACCEL_XOUT_H  0x3B
#define REG_GYRO_XOUT_H   0x43

#define I2C_SCL_Pin   22
#define I2C_SDA_Pin   23

// Private variables
uint8_t mpu_fsm = 0;
uint16_t mpu_gyro_cal_counter = 0;
int32_t mpu_offset_x = 0;
int32_t mpu_offset_y = 0;
int32_t mpu_offset_z = 0;
int16_t mpu_offset_x16 = 0;
int16_t mpu_offset_y16 = 0;
int16_t mpu_offset_z16 = 0;

void mpu_i2c_writeReg8(uint8_t reg, uint8_t data8){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data8);
  Wire.endTransmission();
}

uint8_t mpu_i2c_readReg8(uint8_t reg){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(MPU_ADDR, 1);
  return Wire.read();
}

void mpu_init(){
  Wire.begin(
    I2C_SDA_Pin,
    I2C_SCL_Pin,
    400000
    );  
  delay(1);
  mpu_i2c_writeReg8(
    REG_PWRMGMT_1,
    0x00
    ); 

  mpu_i2c_writeReg8(
    REG_PWRMGMT_1,
    0x00
    );   

  mpu_i2c_writeReg8(
    REG_SMPRT_DIV,
    2 - 1      // -> Sample rate of 500Hz
    );
    
  mpu_i2c_writeReg8(
    REG_DLPFCONF,
    0x03            // -> Accel 44Hz - Gyro 42Hz
    );
  mpu_i2c_writeReg8(
    REG_GYROCONF,
    3 << 3
    );    
  mpu_i2c_writeReg8(
    REG_ACCLCONF,
    3 << 3
    );
}

void mpu_gyroCalRoutine(
  sensor3dRaw_t *gyro_data
  ){
  while(1){
    delay(2);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_GYRO_XOUT_H);
    Wire.endTransmission(false);
  
    Wire.requestFrom(MPU_ADDR, 6);
    gyro_data->sx.sx8.sx8_High  = Wire.read();
    gyro_data->sx.sx8.sx8_Low   = Wire.read();
    gyro_data->sy.sy8.sy8_High  = Wire.read();
    gyro_data->sy.sy8.sy8_Low   = Wire.read();
    gyro_data->sz.sz8.sz8_High  = Wire.read();
    gyro_data->sz.sz8.sz8_Low   = Wire.read();

    mpu_offset_x += (int32_t)((int16_t)gyro_data->sx.sx16);
    mpu_offset_y += (int32_t)((int16_t)gyro_data->sy.sy16);
    mpu_offset_z += (int32_t)((int16_t)gyro_data->sz.sz16);
    
    mpu_gyro_cal_counter++;
    if(mpu_gyro_cal_counter == 2048){
      mpu_gyro_cal_counter = 0;
      mpu_offset_x16 = (int16_t)(mpu_offset_x >> 11);
      mpu_offset_y16 = (int16_t)(mpu_offset_y >> 11);
      mpu_offset_z16 = (int16_t)(mpu_offset_z >> 11);
      break;
    }
  }  
}

void mpu_getData(
  sensor3dRaw_t *gyro_data,
  sensor3dRaw_t *accel_data
  ){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);
  accel_data->sx.sx8.sx8_High  = Wire.read();
  accel_data->sx.sx8.sx8_Low   = Wire.read();
  accel_data->sy.sy8.sy8_High  = Wire.read();
  accel_data->sy.sy8.sy8_Low   = Wire.read();
  accel_data->sz.sz8.sz8_High  = Wire.read();
  accel_data->sz.sz8.sz8_Low   = Wire.read();
  Wire.read();
  Wire.read();
  gyro_data->sx.sx8.sx8_High  = Wire.read();
  gyro_data->sx.sx8.sx8_Low   = Wire.read();
  gyro_data->sy.sy8.sy8_High  = Wire.read();
  gyro_data->sy.sy8.sy8_Low   = Wire.read();
  gyro_data->sz.sz8.sz8_High  = Wire.read();
  gyro_data->sz.sz8.sz8_Low   = Wire.read();
  gyro_data->sx.sx16 = (uint16_t)(((int16_t)gyro_data->sx.sx16) - mpu_offset_x16);
  gyro_data->sy.sy16 = (uint16_t)(((int16_t)gyro_data->sy.sy16) - mpu_offset_y16);
  gyro_data->sz.sz16 = (uint16_t)(((int16_t)gyro_data->sz.sz16) - mpu_offset_z16);
}
