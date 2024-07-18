#define TX2_pin 17
#define RX2_pin 16

typedef struct lidar_data {
  //uint8_t sync_byte;
  uint16_t angle_index;
  uint16_t motor_rpm;
  struct {
    uint16_t intensity;
    uint16_t distance;
    uint16_t reserved;
  } measurement[6];
};

lidar_data hls_data;

void hls_initLiDAR() {
  Serial2.begin(230400, SERIAL_8N1, RX2_pin, TX2_pin);
  Serial2.write('e');
  delay(100);
  Serial2.write('b');
}

uint8_t hls_fsm;
uint8_t hls_idx;
void hls_poll(){
  switch (hls_fsm) {
    case 0:// Detect frame start
      {
        if (0xFA == Serial2.read()) {
          hls_fsm = 1;
        }
      }
      break;

    case 1:
      {
        *(uint8_t *)(&hls_data + hls_idx) = Serial2.read();
        hls_idx++;
        if(hls_idx > 41){
          hls_idx = 0;
          hls_fsm = 0;
        }
      }
      break;
  }
}
