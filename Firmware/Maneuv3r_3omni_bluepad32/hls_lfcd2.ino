#define TX2_pin 17
#define RX2_pin 16

typedef struct __attribute__ ((packed)) lidar_data{
  union{
    uint16_t header;
    struct{
      uint8_t sync_byte :8;
      uint8_t angle_idx :8;
    };
  };
  uint16_t motor_rpm;
  struct {
    uint16_t intensity;
    uint16_t distance;
    uint16_t reserved;
  } measurement[6];
  uint16_t checksum;
};

lidar_data hls_data;

void hls_initLiDAR() {
  Serial2.begin(230400, SERIAL_8N1, RX2_pin, TX2_pin);
//  Serial2.setRxBufferSize();
  Serial2.write('e');
}

uint8_t hls_fsm;
uint8_t hls_idx;
void hls_poll(){
  switch (hls_fsm) {
    case 0:
      {
        Serial2.write('b');
        hls_fsm = 1;
      }
    case 1:// Detect frame start
      {
        if (0xFA == Serial2.read()) {
          hls_idx++;
          hls_fsm = 2;
        }
      }
      break;

    case 2:
      {
        *(uint8_t *)(&hls_data + hls_idx) = Serial2.read();
        hls_idx++;
        if(hls_idx > 41){
          hls_idx = 0;
          hls_fsm = 1;
        }
      }
      break;
  }
}
