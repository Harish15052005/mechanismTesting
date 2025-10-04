#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
// #include "motors.h"
#include <Wire.h>
#include <MPU6050_tockn.h>

const char* SSID = "ROBOCON LDCE";
const char* PASS = "RBCN2025";
const char* broker = "192.168.0.127";

// MQTT
  WiFiClient espClient;
  PubSubClient client(espClient);

  struct Commands {
    bool up, down, left, right;
    int16_t aux1, aux2;
  } cmd;

  int16_t enc1 = 0;
  int16_t enc2 = 0;

  void onMessage(char* topic, byte* payload, unsigned int length)
  {
    if (length != 7) return;           // Invalid packet size
    if (payload[0] != 0xAA) return;    // Invalid header

    uint8_t checksum = 0;
    for (int i = 0; i < 6; i++) checksum ^= payload[i];
    if (checksum != payload[6]) return; // Bad checksum
    
    uint8_t buttons = payload[1];
    cmd.up    = buttons & (1 << 0);
    cmd.down  = buttons & (1 << 1);
    cmd.left  = buttons & (1 << 2);
    cmd.right = buttons & (1 << 3);
    
    cmd.aux1 = payload[2] | (payload[3] << 8);
    cmd.aux2 = payload[4] | (payload[5] << 8);
  }

  void reconnect() {
    while(!client.connected()) {
      if(client.connect("ESP32Robot")) {
        client.subscribe("robot/commands");
      } else {
        delay(1000);
      }
    }
  }

  unsigned long lastTelem = 0;
  void publishTelemetry() {
    // Example: increment encoders for testing
    enc1 += 1;
    enc2 += 2;

    uint8_t payload[4];
    payload[0] = enc1 & 0xFF;
    payload[1] = (enc1 >> 8) & 0xFF;
    payload[2] = enc2 & 0xFF;
    payload[3] = (enc2 >> 8) & 0xFF;

    client.publish("robot/telem", payload, 4);
    // Serial.print(enc1);Serial.print("\t");Serial.print(enc2);Serial.println("\t");
  }
//

MPU6050 mpu(Wire);

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);

  WiFi.begin(SSID, PASS);
  while(WiFi.status() != WL_CONNECTED) {
    digitalWrite(2, !digitalRead(2));
    delay(500);
  }

  client.setServer(broker, 1883);
  client.setCallback(onMessage);

  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets(true,0,0);
}

// float maxYspeed = 50.0*PWM_MOTOR_MAX_DS;
// float maxWspeed = 50.0*PWM_MOTOR_MAX_DS;
float maxYspeed = 50.0;
float maxWspeed = 50.0;

float yaw, yawSetPT;
unsigned long prevTimeDiff = 0;

bool prevRight, prevLeft;

void loop() {
  if(!client.connected()) reconnect();
  client.loop();

  mpu.update();
  yaw = mpu.getAngleZ();

  int16_t vy = (cmd.up - cmd.down)*maxYspeed;
  
  yawSetPT += 90 * ( (cmd.right && !prevRight)  - (cmd.left && !prevLeft) );
  prevRight = cmd.right;
  prevLeft = cmd.left;

  unsigned long currentTime = micros();
  float timedeff = (currentTime - prevTimeDiff)/1000000.0;  
  prevTimeDiff = currentTime;
  int16_t w = yawPd(timedeff);

  tankDrive(vy,w);

  if(millis() - lastTelem > 100) { // 10 Hz telemetry
    lastTelem = millis();
    publishTelemetry();
  }

  Serial.print(yawSetPT);

  Serial.println();
  // Serial.printf("U:%d D:%d L:%d R:%d A1:%d A2:%d\n",
                // cmd.up, cmd.down, cmd.left, cmd.right, cmd.aux1, cmd.aux2);
}


void tankDrive(int16_t vy, int16_t w)
{
  // setMotorSpeed(lpwm, ldir, vy + w);
  // setMotorSpeed(rpwm, rdir, vy - w);
}

int16_t yawPd(float timedeff){
  return 4096;
}
