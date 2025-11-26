#include<M5Stack.h>
//#include "sdk/include/angles.h"
#include "doProcess.h"
#include "mapData.h"
#include "espnow.h"
//#include <espnow.h>/
#include "espHttpServer.h"
#include "X2driver.h"
#include "lidarcar.h"
#include "lock.h"
#include <Arduino.h>
#include <stdio.h>
#include <Wire.h>
#include <SPI.h>
#define TFT_GREY 0x5AEB // New colour

// Set to 0 to disable ESP-NOW remote controller support and use only
// wired (USB Serial) + keyboard control. Set to 1 to keep remote behaviour.
#define USE_REMOTE 0

int state = 0;

EspNowMaster espnow;
HttpServer httpServer;
X2 lidar;
LidarCar lidarcar;
Preferences preferences;

void setLcd() {
  M5.Lcd.clear(BLACK);
//  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(RED);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(3, 10);
  M5.Lcd.println("X2 LidarBot");
  M5.Lcd.drawCircle(160, 120 , 4, RED);
}

void printPeerList() {
  setLcd();
  M5.Lcd.setCursor(0, 50);
  M5.Lcd.printf("slave list:\r\n");
  for(int i = 0; i < espnow.peerlist.count; i++) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           espnow.peerlist.list[i].peer_addr[0], 
           espnow.peerlist.list[i].peer_addr[1],
           espnow.peerlist.list[i].peer_addr[2],
           espnow.peerlist.list[i].peer_addr[3],
           espnow.peerlist.list[i].peer_addr[4],
           espnow.peerlist.list[i].peer_addr[5]);
    M5.Lcd.println(macStr);
    M5.Lcd.println();
  }
}

void readAddressFromRom(){
  esp_now_peer_info_t slave_set;
  uint8_t get_len = sizeof(esp_now_peer_info_t);
  if(preferences.getBytes("mac_addr1", &slave_set, get_len) == get_len)
  {
    Serial.printf("<<<<<<<--->>>>>>> \r\n");
    espnow.confirmPeer(slave_set);
  }
}



void disPlay(void) {
  float map_data_stash[720] = { 0.0 };
  xSemaphoreTake( xSemaphore, portMAX_DELAY);
  memcpy(map_data_stash, lidar.dismap.mapdata, 720*sizeof(float));
  xSemaphoreGive( xSemaphore );

  for (int i = 0; i < 720; i++) {
      float oldAng = from_degrees((i / 2.0));
      float Ang = from_degrees((i / 2.0));

      float oldX = (sin(oldAng) * lidar.oldmap.mapdata[i]/5);
      float oldY = (cos(oldAng) * lidar.oldmap.mapdata[i]/5);
      float X = (sin(Ang) * map_data_stash[i]/5);
      float Y = (cos(Ang) * map_data_stash[i]/5);

      if (lidar.oldmap.mapdata[i] > 0) {
        M5.Lcd.drawPixel(-oldX + 160, oldY + 120, BLACK);  
      }

      if (map_data_stash[i] > 0) {
        M5.Lcd.drawPixel(-X + 160, Y + 120, WHITE);  
      }
      lidar.oldmap.mapdata[i] = map_data_stash[i];
  }
}

void sendMap() {
//   ScanForSlave();

    // If Slave is found, it would be populate in `slave` variable
  // We will check if `slave` is defined and then we proceed further
  if (espnow.slave.channel == CHANNEL) { // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
    bool isPaired = espnow.manageSlave();
     if (isPaired) {
      // pair success or already paired
      // Send data to device

        uint8_t head[2] = {0xaa, 0x55};
        espnow.sendData(head, 2);
        for (int i = 0; i < 6; i++) {
          espnow.sendData(lidar.tmpData.mapdata + i * 120, sizeof(uint16_t) * 120);
        }
        memset(lidar.tmpData.mapdata, 0, 1500);
        
    } else {
      // slave pair failed
      Serial.println("Slave pair failed!");
    }
  }
  else {
    // No slave found to process
  }
}

int lastCount = 0;
static void dis_task(void *arg) {
  Serial.printf("uart_task\r\n");
  while(1) {
    if (espnow.isConnected == true) {
      if (lidar.disPlayFlag){
        sendMap();
        disPlay();
        delay(5); 
        lidar.disPlayFlag = 0;
      }
    } else {
      Serial.printf("peer count: %d\r\n", espnow.peerlist.count);
      if (espnow.peerlist.count > lastCount) {
        printPeerList();     
        lastCount = espnow.peerlist.count;
      }
    }
    
    delay(10);
  }

}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  // Serial.print("Last Packet Recv Data: "); Serial.println(*data);
  // Serial.println("");
  static uint8_t data_last[3] = {0xff, 0xff, 0xff};
  bool update = false;

  if((data_len == 3)) {
    for(uint8_t i = 0; i < 3; i++)
    {
      if(data_last[i] != data[i])
      {
        data_last[i] = data[i];
        update = true;
      }
    }
    
    if(update)
    {
      lidarcar.ControlWheel(data[0], data[1], data[2]);
    }
  }
}

int choice = 0;

void setup() {
  Serial.begin(115200);
  // Enable M5 core and enable internal Serial so USB cable works for Serial Monitor.
  // Previously this used `M5.begin(true, false, false)` which disables the M5 Serial
  // interface and can prevent the board talking over the USB cable. Enable the
  // serial argument (third param) so `Serial` is connected to the USB/Serial port.
  M5.begin(true, false, true);
  
  // put your setup code here, to run once:
//  BtnSet();
  Serial1.begin(115200, SERIAL_8N1, 16, 17);
  Serial2.begin(115200);

   
  setLcd();
  //Serial.printf("got crc %x\r\n", cover_crc(msg_data, data, 80));

  // Preferences are used for storing paired peer address — keep available
  preferences.begin("lidarBot", false);

  if (USE_REMOTE) {
    espnow.Init();

    if (!M5.BtnC.isPressed()){
      readAddressFromRom();
    }

    espnow.setRecvCallBack(&OnDataRecv);
    // espnow.setSendCallBack(&OnDataSent);
  } else {
    // If remote is disabled, ignore BtnC pairing behavior.
  }

  httpServer.init();

  // Start the lidar display task in any case (it doesn't require remote control).
  xTaskCreatePinnedToCore(dis_task, "lidar", 10 * 1024, NULL, 1, NULL, 1);
}

// Helper: send control to peer (if connected) and apply locally
void sendControlCommand(int8_t X, int8_t Y, uint8_t A) {
  // Apply locally
  Serial.println("Local control:");
  lidarcar.ControlWheel(X, Y, A);

  // Forward to peer if connected so a remote unit can mirror the command
  if (USE_REMOTE) {
    if (espnow.isConnected) {
      uint8_t buf[3];
      buf[0] = (uint8_t)X;
      buf[1] = (uint8_t)Y;
      buf[2] = A;
      espnow.sendData(buf, 3);
    }
  }
}
// Optional: map WASD (and X to stop) from USB Serial to drive commands.
// Call this from your Serial Monitor by sending single characters.
void processKeyboardControl() {
  while (Serial.available()) {
    int p = Serial.peek();
    if (p == -1) break;
    char c = (char)p;

    // Single-character control (WASD/X)
    if (c == 'w' || c == 'W' || c == 'a' || c == 'A' || c == 's' || c == 'S' || c == 'd' || c == 'D' || c == 'x' || c == 'X') {
      Serial.read(); // consume
      int8_t X = 0;
      int8_t Y = 0;
      uint8_t A = 0;
      switch (c) {
        case 'w': case 'W': X = 0; Y = 2; A = 0; break;
        case 's': case 'S': X = 0; Y = -2; A = 0; break;
        case 'a': case 'A': X = -2; Y = 1; A = 0; break;
        case 'd': case 'D': X = 2; Y = 1; A = 0; break;
        case 'x': case 'X': X = 0; Y = 0; A = 0; break;
      }
      Serial.printf("KB cmd: %c -> X=%d Y=%d A=%d\r\n", c, X, Y, A);
      sendControlCommand(X, Y, A);
    } else {
      // Try to read a numeric wire command line like: "0 2 0" or "-1 3 0"
      String line = Serial.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) continue;
      int xi = 0, yi = 0, ai = 0;
      if (sscanf(line.c_str(), "%d %d %d", &xi, &yi, &ai) == 3) {
        Serial.printf("Wire cmd -> X=%d Y=%d A=%d\r\n", xi, yi, ai);
        sendControlCommand((int8_t)xi, (int8_t)yi, (uint8_t)ai);
      } else {
        Serial.println("Unrecognized input. Send 'w/a/s/d/x' or a line: 'X Y A'");
      }
    }
  }
}

void loop() {
  M5.update();
  if (USE_REMOTE) {
    espnow.Broadcast();
  }

  if (USE_REMOTE) {
    if (M5.BtnB.wasPressed()) {
      espnow.confirmPeer(espnow.peerlist.list[choice]);
      preferences.putBytes("mac_addr1", &espnow.peerlist.list[choice], sizeof(esp_now_peer_info_t));
      setLcd();
      delay(100);
    }

    if (!espnow.isConnected) {
      if(M5.BtnA.wasPressed()) {
        choice--;
        if (choice < 0) {
          choice = 0;
        }
        //setLcd();
        printPeerList();
        M5.Lcd.fillCircle(220, choice*15 + 80, 3, RED);
      } else if(M5.BtnC.wasPressed()) {
        choice++;
        if (choice > espnow.peerlist.count - 1) {
          choice = espnow.peerlist.count - 1;
        }
        //setLcd();
        printPeerList();
        M5.Lcd.fillCircle(220, choice*15 + 80, 3, RED);
      }
    }
  } else {
    // Remote disabled: optionally map M5 buttons to local control (quick presets)
    if (M5.BtnA.wasPressed()) {
      // turn left in place
      sendControlCommand(-2, 1, 0);
    }
    if (M5.BtnB.wasPressed()) {
      // stop
      sendControlCommand(0, 0, 0);
    }
    if (M5.BtnC.wasPressed()) {
      // turn right in place
      sendControlCommand(2, 1, 0);
    }
  }
	

  while (Serial1.available()) {
    lidar.lidar_data_deal(Serial1.read());
  }
  processKeyboardControl();
  delay(10);
}

