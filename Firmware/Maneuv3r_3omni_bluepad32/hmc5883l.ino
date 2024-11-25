// mag sensor
#define SCL_PIN       22
#define SDA_PIN       21
#define I2C_SPEED     400000

#define HMC5883L_ADDR 0x1E
#define MAG_XH        0x03
#define MAG_YH        0x07
#define MAG_ZH        0x05

int16_t mx, my;

float pos_z_offset;

uint8_t hmc5883l_readReg8(uint8_t reg) {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(HMC5883L_ADDR, 1);
  return Wire.read();
}

void hmc5883l_writeReg(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void hmc5883l_init(){
  hmc5883l_writeReg(0x00, 0x78);// 75Hz sampling rate and average two sample
  hmc5883l_writeReg(0x02, 0x01);// Single measurement mode
}

void hmc5883l_getInitialPosZ(){
    // get first offset pose
  mx = hmc5883l_readReg8(MAG_XH) << 8;
  mx |= hmc5883l_readReg8(MAG_XH+1);
  my = hmc5883l_readReg8(MAG_YH) << 8;
  my |= hmc5883l_readReg8(MAG_YH+1); 
  
  hmc5883l_readReg8(MAG_ZH);
  hmc5883l_readReg8(MAG_ZH+1);  

  hmc5883l_writeReg(0x02, 0x01);// Single measurement mode

  pos_z_offset = atan2pi((float)my/65535.0f, (float)mx/65535.0f);// Unit Rad  
}

void hmc5883l_getPosZ(odometry_t *robot_odomptr){
    mx = hmc5883l_readReg8(MAG_XH) << 8;
    mx |= hmc5883l_readReg8(MAG_XH+1);
    my = hmc5883l_readReg8(MAG_YH) << 8;
    my |= hmc5883l_readReg8(MAG_YH+1); 
    
    hmc5883l_readReg8(MAG_ZH);
    hmc5883l_readReg8(MAG_ZH+1);  

    hmc5883l_writeReg(0x02, 0x01);// Single measurement mode

    robot_odomptr->pos_abs_az =
//      pos_z_offset - 
     atan2pi((float)my, (float)mx);// Unit Rad
}
