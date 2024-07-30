// Bluetooth controller's MAC address
bd_addr_t controller_addr = {0xE0, 0xF8, 0x48, 0x05, 0x00, 0x61};

uint8_t bluepad32_con_flag = 0;

void bluepad32_ConnectedCallback(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    if (myControllers == nullptr) {
        Serial.printf("CALLBACK: Controller is connected");
        ControllerProperties properties = ctl->getProperties();
        Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                       properties.product_id);
        myControllers = ctl;
        foundEmptySlot = true;
        bluepad32_con_flag = 1;
        return;
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void bluepad32_DisconnectedCallback(ControllerPtr ctl) {
    bool foundController = false;

    if (myControllers == ctl) {
        Serial.printf("CALLBACK: Controller disconnected");
        myControllers = nullptr;
        foundController = true;
        bluepad32_con_flag = 0;
        return;
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

uint8_t bluepad32_getConnectStatus(){
  return bluepad32_con_flag;  
}

void bluepad32_Init(){
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&bluepad32_ConnectedCallback, &bluepad32_DisconnectedCallback);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
  
  uni_bt_allowlist_add_addr(controller_addr);
  uni_bt_allowlist_set_enabled(true);
}

float cmdx, cmdy;
uint16_t mask_btn = 0;
uint16_t bluepad32_processControllers(headvel_t *joyvel_headcmd) {
  bool dataUpdated = BP32.update();
    if (!dataUpdated){
      goto exit_ret;
    }
    
    if (myControllers && myControllers->isConnected() && myControllers->hasData()) {
        if (myControllers->isGamepad()) {
            cmdx = (float)(map(myControllers->axisY(), -511, 512, 25, -25))/100.0;
            cmdy = (float)(map(myControllers->axisX(), -511, 512, 25, -25))/100.0;

            joyvel_headcmd->heading = atan2pi(cmdy, cmdx);
            
            cmdx = cmdx * cmdx;
            cmdy = cmdy * cmdy;
            joyvel_headcmd->vel = constrain(sqrt(cmdx + cmdy), -0.25, 0.25);
            joyvel_headcmd->vel -= 0.01;
            if((abs(joyvel_headcmd->vel) > -0.005) && (abs(joyvel_headcmd->vel) < 0.005))
              joyvel_headcmd->vel = 0.0;

            joyvel_headcmd->vaz = (float)(map(myControllers->axisRX(), -511, 512, 18, -18))/10.0;
            joyvel_headcmd->vaz -= 0.1;
            if((abs(joyvel_headcmd->vaz) > -0.05) && (abs(joyvel_headcmd->vaz) < 0.05))
              joyvel_headcmd->vaz = 0.0;

//            Serial.printf("Heading: %.05f", joyvel_headcmd->heading);
//            Serial.printf("Magnitude: %.05f", joyvel_headcmd->vel);
//            Serial.printf("Angular: %.05f\n", joyvel_headcmd->vaz);
//            
//            Serial.printf(
//                "dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
//                "misc: 0x%02x\n",
//                myControllers->dpad(),         // D-pad
//                myControllers->buttons(),      // bitmask of pressed buttons
//                myControllers->axisX(),        // (-511 - 512) left X Axis
//                myControllers->axisY(),        // (-511 - 512) left Y axis
//                myControllers->axisRX(),       // (-511 - 512) right X axis
//                myControllers->axisRY(),       // (-511 - 512) right Y axis
//                myControllers->brake(),        // (0 - 1023): brake button
//                myControllers->throttle(),     // (0 - 1023): throttle (AKA gas) button
//                myControllers->miscButtons()  // bitmask of pressed "misc" buttons
//            );

            mask_btn = (((myControllers->throttle() > 1) ? 1 : 0) << 15) | ((myControllers->buttons() << 4) | myControllers->dpad());
            Serial.printf("btn mask : 0x%04X\n", mask_btn);

            return mask_btn;
        }
    }
    
exit_ret:
  joyvel_headcmd->vel = 0.0;
  joyvel_headcmd->heading = 0.0;
  joyvel_headcmd->vaz = 0.0;
  
  return 0x0000;
}
