#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

#define SERVO_PAN_PIN 10
#define SERVO_TILT_PIN 9
#define CE_PIN 7
#define CSN_PIN 8
#define LASER 3

#define RADIO_TEST_DATA 4286

Servo servo_pan; // сервопривод вокруг вертикальной оси (влево-вправо)
Servo servo_tilt; // сервопривод вокруг горизонтальной оси (вверх-вниз)

RF24 radio(CE_PIN, CSN_PIN);
const byte RX_ADDR[6] = "RX001";
const byte WX_ADDR[6] = "TX001";

const uint8_t DEVICE_ID = 1;

const float INIT_ANGLE_TILT = 101.75; //откалиброванные значения
const float MIN_ANGLE_TILT = INIT_ANGLE_TILT - 40;
const float MAX_ANGLE_TILT = INIT_ANGLE_TILT + 40;
const int STEP_ANGLE = 10;
const float INIT_ANGLE_PAN = 93; //откалиброванные значения
const float MIN_ANGLE_PAN = INIT_ANGLE_PAN - 40;
const float MAX_ANGLE_PAN = INIT_ANGLE_PAN + 40;
const int STEP_INTERVAL = 3000;

float pan;
float tilt;
int8_t mode = 0;
// 0-ожидание
// 1-горизонтальный скан
// 2-вертикальный скан
// 3-диагональный скан №1
// 4-диагональный скан №2

bool connection = 0;
const int32_t test_data = 6824;

unsigned long lastStepTime = 0;

struct status_msg {
  // статус cubsat, передаваемый по радиоканалу
  uint8_t id;
  int8_t tilt;
  int8_t pan;
  int8_t mode;
};

struct command_msg {
  // команды для cubesat, передаваемые по радиоканалу
  int8_t cmd; // 1 - START; -1 - STOP
};

status_msg status;
command_msg command;

void moveServos() {
  // изменение положения сервоприводов
  servo_pan.write(pan);
  delay(10);
  servo_tilt.write(tilt);
}

void sendStatus() {
  // отправка статуса устройства через радиомодуль nRF24
  status.id = DEVICE_ID;
  status.tilt = tilt-INIT_ANGLE_TILT;
  status.pan = pan-INIT_ANGLE_PAN;
  status.mode = mode;

  radio.stopListening();
  radio.write(&status, sizeof(status));
  radio.startListening();

  Serial.print("MODE ");
  Serial.print(status.mode);
  Serial.print(" | TILT ");
  Serial.print(status.tilt);
  Serial.print(" | PAN ");
  Serial.println(status.pan);
}

bool checkChannel(uint8_t channel){
  //проверка, передаются ли тестовые данные по выбранному каналу
  radio.setChannel(channel);
  radio.startListening();
  int32_t r_data = 0;
  for (int i = 0; i < 2048; i++){
    if (radio.available()){
      radio.read(&r_data, sizeof(r_data));
      if (r_data == RADIO_TEST_DATA){
        radio.stopListening();
        for (int j = 0; j < 2048; j++){
          if (radio.write(&test_data, sizeof(test_data))) return 1; 
        }
      }
    }
  }
  radio.stopListening();
  return 0;
}

bool servoBITCube(){
  // тестирование сервоприводов
  pan = MAX_ANGLE_PAN;
  tilt = MAX_ANGLE_TILT;
  moveServos();
  delay(500);
  if (abs(servo_tilt.read() - tilt) > 1) return 0;
  pan = MIN_ANGLE_PAN;
  tilt = MIN_ANGLE_TILT;
  moveServos();
  delay(500);
  if (abs(servo_tilt.read() - tilt) > 1) return 0;
  return 1;  
}

void laserTest(){
  // тестирование лазера
  for(int8_t i = 1; i <= 3; i++){
      digitalWrite(LASER, HIGH);
      delay(3000);
      digitalWrite(LASER, LOW);
      delay(3000);
  }
}

void setup() {
  Serial.begin(9600);

  // установка пинов для лазерного указателя, аварийной кнопки и сервоприводов
  pinMode(LASER, OUTPUT);
  servo_tilt.attach(SERVO_TILT_PIN);
  servo_pan.attach(SERVO_PAN_PIN);
  delay(2000);
  if (servoBITCube()){
    // установка сервоприводов в начальное положение
    tilt = INIT_ANGLE_TILT;
    pan = INIT_ANGLE_PAN;
    mode = 0;
    moveServos();
    laserTest();
  } else mode = -2;
  // настройка радиомодуля nRF24
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.openReadingPipe(0, RX_ADDR);
  radio.openWritingPipe(WX_ADDR);
  Serial.println("Waiting for connection.");

  //Перебор каналов и поиск рабочего канала (канала передачи тестовых данных)
  for(uint8_t channel = 1; !connection; channel++){
    if (channel == 126) channel = 0;
    connection = checkChannel(channel);
  }
  radio.startListening();
  
  if(!mode) Serial.println("Ready. Waiting for command."); else Serial.println("Servo BIT failure.");
}

void loop() {
  if (radio.available() && (mode >= 0)) {
    // чтение команды по радиоканалу
    radio.read(&command, sizeof(command));
    switch (command.cmd) {
      case 1:
        // обработка команды начала сканирования
        mode = 1;
        tilt = MIN_ANGLE_TILT;
        pan = INIT_ANGLE_PAN;
        lastStepTime = millis();
        break;
      case -1:
        // обработка команды аварийной остановки
        mode = -1;
        break;
    }
  }

  if (millis() - lastStepTime < STEP_INTERVAL) return;
  lastStepTime = millis();

  switch (mode) {
    case -1:
      // аварийная остановка
      digitalWrite(LASER, LOW);
      tilt = INIT_ANGLE_TILT;
      pan = INIT_ANGLE_PAN;
      moveServos();
    case -2:
      //провал тестирования сервоприводов
      sendStatus();
      for (;;);
      break;
    case 1:
      // горизонтальный скан
      digitalWrite(LASER, HIGH);
      moveServos();
      sendStatus();
      tilt += STEP_ANGLE;

      if (tilt > MAX_ANGLE_TILT) {
        tilt = INIT_ANGLE_TILT;
        pan = MIN_ANGLE_PAN;
        mode = 2;
      } 
      break;
    case 2:
      // вертикальный скан
      moveServos();
      sendStatus();
      pan += STEP_ANGLE;

      if (pan > MAX_ANGLE_PAN) {
        tilt = MIN_ANGLE_TILT;
        pan = MIN_ANGLE_PAN;
        mode = 3;
      }
      break;
    case 3:
      // диагональный скан №1
      moveServos();
      sendStatus();
      tilt += STEP_ANGLE;
      pan += STEP_ANGLE;

      if (tilt > MAX_ANGLE_TILT) {
        tilt = MIN_ANGLE_TILT;
        pan = MAX_ANGLE_PAN;
        mode = 4;
      }
      break;
    case 4:
      // диагональный скан №2
      moveServos();
      sendStatus();
      tilt += STEP_ANGLE;
      pan -= STEP_ANGLE;

      if (tilt > MAX_ANGLE_TILT) {
        tilt = INIT_ANGLE_TILT;
        pan = INIT_ANGLE_PAN;
        mode = 0;
        sendStatus();
      }
      break;
    case 0:
    default:
      tilt = INIT_ANGLE_TILT;
      pan = INIT_ANGLE_PAN;
      mode = 0;
      moveServos();
      digitalWrite(LASER, LOW);
      break;
  }
}