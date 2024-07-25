// Gyro sensor
#define SCL_PIN       22
#define SDA_PIN       21
#define I2C_SPEED     400000

#define ITG_TO_RAD    0.00121414f // (2000/28750 degree*s^-1/LSB) * 0.0174533 (rad/degree)

#define ITG3205_ADDR  0x68

#define REG_WAI       0x00
#define REG_SMPDIV    0x15
#define REG_DLPF      0x16
#define REG_INTCFG    0x17
#define REG_INTSR     0x1A

#define REG_TMP_H     0x1B
#define REG_GYX_H     0x1D
#define REG_GYY_H     0x1F
#define REG_GYZ_H     0x21      

#define REG_PWRMGM    0x3E
uint8_t itg3205_readReg8(uint8_t reg) {
  Wire.beginTransmission(ITG3205_ADDR);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(ITG3205_ADDR, 1);
  return Wire.read();
}

void itg3205_writeReg(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(ITG3205_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void itg3205_init(){
  // Sensor probe
  if(!(itg3205_readReg8(REG_WAI) & ITG3205_ADDR))
    while(1);
    
  itg3205_writeReg(REG_DLPF, (3 << 3) | 1);// Set gyro full scale and 188Hz DLPF
  itg3205_writeReg(REG_SMPDIV, 7);// Set data out rate at 125hz
  itg3205_writeReg(REG_PWRMGM, 3);// Set clock source to PLL with Z Gyro reference

  itg3205_calib();
}

float itg3205_getVz(){
  int16_t gz = 0;
  gz = itg3205_readReg8(REG_GYZ_H) << 8;
  gz |= itg3205_readReg8(REG_GYZ_H + 1);

  return gz;
}
float gyro_offset;
void itg3205_calib(){
  int32_t gyro_raw;
  for(uint16_t i=0; i < 2000; i++){
      gyro_raw += itg3205_getVz();
      delay(1);
  }  
  gyro_offset = (float)(gyro_raw)/2000.0;
}

void itg3205_getPosZ(odometry_t *robot_odomptr){
  robot_odomptr->vel_az = (gyro_offset - itg3205_getVz()) * ITG_TO_RAD;// Gyro mounted upside-down, so the offset is subtracting the reading
  //robot_odomptr->pos_az += robot_odomptr->vel_az * 0.008;
}
