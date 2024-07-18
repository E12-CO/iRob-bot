#define REG_PRODID  0x00 // Use to verify the Serial communication
#define REG_MOTION  0x02 // Motion detection register
#define REG_DELTAX  0x03 // Delta X movement 
#define REG_DELTAY  0x04 // Delta Y movement
#define REG_GETPIX  0x0B // Get Raw pixel reading
#define REG_MCTRL2  0x10 // Mouse control 2 (setting DPI)
#define REG_RDBRST  0x63 // Motion Burst, Read Delta X and Y in single command
#define REG_LEDCTR  0x22 // LED control (strobe mode or always on)

typedef struct pin_t{
  int ncs;
  int sio;
  int sck;
};

pin_t adns_Pin_t;

void adns5050_init(int ncs, int sio, int sck) {
  // Setup GPIO
  adns_Pin_t.ncs = ncs;
  adns_Pin_t.sio = sio;
  adns_Pin_t.sck = sck;

  pinMode(adns_Pin_t.ncs, OUTPUT);
  digitalWrite(adns_Pin_t.ncs, HIGH);
  pinMode(adns_Pin_t.sio, INPUT);
  pinMode(adns_Pin_t.sck, OUTPUT);
}

// Bit-banging register 8 bit read
uint8_t adns5050_readReg(uint8_t reg) {
  uint8_t ret = 0;

  digitalWrite(adns_Pin_t.ncs, LOW);
  pinMode(adns_Pin_t.sio, OUTPUT);
  for (uint8_t i = 0; i < 8; i ++) {
    digitalWrite(adns_Pin_t.sio, (reg && 0x80) ? HIGH : LOW);
    digitalWrite(adns_Pin_t.sck, HIGH);
    digitalWrite(adns_Pin_t.sck, LOW);
    reg <<= 1;
  }
  pinMode(adns_Pin_t.sio, INPUT);
  delayMicroseconds(4);

  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(adns_Pin_t.sck, HIGH);
    ret |= digitalRead(adns_Pin_t.sio);
    digitalWrite(adns_Pin_t.sck, LOW);
    ret <<= 1;
  }

  digitalWrite(adns_Pin_t.ncs, HIGH);

  return ret;
}

// Bit-banging register 8 bit write
void adns5050_writeReg(uint8_t reg, uint8_t val) {
  reg |= (1 << 7);// Set MSB bit to indicate register writing access.

  digitalWrite(adns_Pin_t.ncs, LOW);
  pinMode(adns_Pin_t.sio, OUTPUT);
  for (uint8_t i = 0; i < 8; i ++) {
    digitalWrite(adns_Pin_t.sio, (reg && 0x80) ? HIGH : LOW);
    digitalWrite(adns_Pin_t.sck, HIGH);
    digitalWrite(adns_Pin_t.sck, LOW);
    reg <<= 1;
  }
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(adns_Pin_t.sio, (val && 0x80) ? HIGH : LOW);
    digitalWrite(adns_Pin_t.sck, HIGH);
    digitalWrite(adns_Pin_t.sck, LOW);
    val <<= 1;
  }
  pinMode(adns_Pin_t.sio, INPUT);
  digitalWrite(adns_Pin_t.ncs, HIGH);
}

// Probe for sensor by reading the sensor ID
uint8_t adns5050_Probe() {
  return adns5050_readReg(REG_PRODID) != 0x12;
}

// Turn Lighting LED on 
void adns5050_setLED() {
  adns5050_writeReg(REG_LEDCTR, 0x80);
}

// Set Count Per Inch to 625 Count per Inch
void adns5050_set625cpi(){
  adns5050_writeReg(REG_MCTRL2, 0x10);// First, enable the RES register
  adns5050_writeReg(REG_MCTRL2, 0x15);// Select 625CPI
}

// Read the travel distance of X and Y axis
// The readings are in Count per Inch unit,
// Inch to meter conversion will be perform separately 
void adns5050_getdXdY(int8_t *dX, int8_t *dY) {
  uint8_t reg, ret = 0;

  reg = REG_RDBRST;
  
  digitalWrite(adns_Pin_t.ncs, LOW);
  pinMode(adns_Pin_t.sio, OUTPUT);
  for (uint8_t i = 0; i < 8; i ++) {
    digitalWrite(adns_Pin_t.sio, (reg && 0x80) ? HIGH : LOW);
    digitalWrite(adns_Pin_t.sck, HIGH);
    digitalWrite(adns_Pin_t.sck, LOW);
    reg <<= 1;
  }
  pinMode(adns_Pin_t.sio, INPUT);
  delayMicroseconds(4);

  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(adns_Pin_t.sck, HIGH);
    ret |= digitalRead(adns_Pin_t.sio);
    digitalWrite(adns_Pin_t.sck, LOW);
    ret <<= 1;
  }
  *dX = ret;

  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(adns_Pin_t.sck, HIGH);
    ret |= digitalRead(adns_Pin_t.sio);
    digitalWrite(adns_Pin_t.sck, LOW);
    ret <<= 1;
  }
  *dY = ret;

  digitalWrite(adns_Pin_t.ncs, HIGH);
}

int8_t dx, dy;
void adns5050_updateVel(odometry_t *robot_odom){
    adns5050_getdXdY(&dx, &dy);

    // At 1000 CPI, still need calibration tho, as I will use different optics assembly.
    //vx = (vx/625) * 125 * 0.0254;// CPI->Inch->InchPerSec->MeterPerSec
    //vy = (vy/625) * 125 * 0.0254;// CPI->Inch->InchPerSec->MeterPerSec
    
    robot_odom->vel_x = dx * 0.003175;
    robot_odom->vel_y = dy * 0.003175;
}
