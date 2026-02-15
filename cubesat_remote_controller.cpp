#include <SPI.h>
#include <RF24.h>

#define CE_PIN 7
#define CSN_PIN 8

#define SCOPE_WIDTH 6
#define RADIO_TEST_DATA 6824

RF24 radio(CE_PIN, CSN_PIN);

const byte RX_ADDR[6] = "TX001"; // от CubeSat
const byte WX_ADDR[6] = "RX001"; // к CubeSat

bool connection = 0;
const int32_t test_data = 4286;

struct status_msg {
  uint8_t id;
  int8_t tilt;
  int8_t pan;
  int8_t mode;
};

struct command_msg {
  int8_t cmd; // 1 - START, -1 - STOP
};

status_msg status;
command_msg command;

uint8_t clearChannel(){
  //поиск свободного канала
  radio.stopListening();
  radio.setAutoAck(false);
  uint16_t resp;
  int8_t counter = SCOPE_WIDTH;
  for (uint8_t channel = 60;; channel++){
    if (channel == 126){
      channel = 0;
      counter = SCOPE_WIDTH;
    }
    resp = 0;
    for (int i = 0; i < 2000; i++){
      radio.setChannel(channel);
      radio.startListening();
      delayMicroseconds(500);
      radio.stopListening();
      if (radio.testCarrier()){
        resp++;
      }
    }
    if (!resp){
      counter--;
    } else {
      counter = SCOPE_WIDTH;
    }
    if (!counter){
      radio.setAutoAck(true);
      radio.setChannel(channel-2);
      radio.startListening();
      return (channel-2);
    }
  }
}

bool sendTestData() {
  //отправка тестовых данных
  if (radio.write(&test_data, sizeof(test_data))){
    int32_t r_data = 0;
    radio.startListening();
    for (int i = 0; i < 2048; i++){
      if (radio.available()){
        radio.read(&r_data, sizeof(r_data));
        if (r_data == RADIO_TEST_DATA) return 1;
      }
    }
    radio.stopListening();
  }
  return 0;
}

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.print("Channel: ");
  Serial.println(clearChannel());
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(0, RX_ADDR);
  radio.openWritingPipe(WX_ADDR);
  radio.stopListening();
  while (!connection){
    connection = sendTestData();
  }
  Serial.println("Connected.");  
  radio.startListening();
  Serial.println("s - START, x - STOP");
}

void sendCommand(int8_t cmd) {
  command.cmd = cmd;
  radio.stopListening();
  radio.write(&command, sizeof(command));
  radio.startListening();

  if (cmd == 1) Serial.println("Command sent: START");
  if (cmd == -1) Serial.println("Command sent: STOP");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's') sendCommand(1);      
    if (c == 'x') sendCommand(-1);
  }

  if (radio.available()) {
    radio.read(&status, sizeof(status));
    Serial.print("ID: "); Serial.print(status.id);
    Serial.print(" | TILT: "); Serial.print(status.tilt);
    Serial.print(" | PAN: "); Serial.print(status.pan);
    Serial.print(" | MODE: "); Serial.println(status.mode);

    if (status.mode == -1) Serial.println("CubeSat STOPPED");
    return;
  }
}