// Nama: Gayuh Mukti Aji
// NIM: 25/566296/SV/27058

#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>

// inisialisasi sensor mpu6050
Adafruit_MPU6050 mpu;
// inisialisasi servo
Servo servo1, servo2, servo3, servo4, servo5;

// variable untuk perubahan yaw
float yawBefore = 0;
unsigned long lastStableTime = 0;
bool servo5Back = false;
int counter = 0;

#define threshold 0.01 // nilai batas perubahan yaw yg dianggap gerakan

// pin setiap komponen
#define PIN_PIR 27
#define PIN_SERVO1 23
#define PIN_SERVO2 22
#define PIN_SERVO3 14
#define PIN_SERVO4 12
#define PIN_SERVO5 13
#define PIN_SCL 19
#define PIN_SDA 21

void setup() {
  Serial.begin(115200);

  Wire.begin(PIN_SDA, PIN_SCL);

  // cek sensor mpu6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 tidak terdeteksi!");
    while (1);
  }
  Serial.println("MPU6050 detected!");

  // set pin input PIR
  pinMode(PIN_PIR, INPUT);

  // frekuensi PWM servo
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  servo4.setPeriodHertz(50);
  servo5.setPeriodHertz(50);

  // connect servo ke masing masing pin
  servo1.attach(PIN_SERVO1);
  servo2.attach(PIN_SERVO2);
  servo3.attach(PIN_SERVO3);
  servo4.attach(PIN_SERVO4);
  servo5.attach(PIN_SERVO5);

  // posisi awal servo
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  servo5.write(90);
}

void loop() {

  // variable sensor MPU (accelerometer, gyro, temperature)
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // data rotasi
  float roll = g.gyro.x;
  float pitch = g.gyro.y;
  float yaw = g.gyro.z;

  // ambil status pir
  int statusPIR = digitalRead(PIN_PIR);

  // kondisi pir aktif
  if (statusPIR == HIGH) {
    Serial.println("PIR Trigger!");
    // servo bergerak serentak sebagai respon deteksi gerakan
    servo1.write(50);
    servo2.write(50);
    servo3.write(50);
    servo4.write(50);
    servo5.write(50);
    delay(1000);

    // kembali ke posisi default
    servo1.write(90);
    servo2.write(90);
    servo3.write(90);
    servo4.write(90);
    servo5.write(90);
  }
  else { // mode sensor mpu
    // mapping gyro menjadi derajat servo
    int sudut12 = map(roll, -4.36, 4.36, 180, 0); // berlawanan arah
    int sudut34 = map(pitch, -4.36, 4.36, 0, 180); // searah
    int sudut5 = map(yaw, -4.36, 4.36, 0, 180); // searah

    // servo 1 dan 2 bergerak sesuai dengan x axis
    servo1.write(sudut12);
    servo2.write(sudut12);
    // servo 3 dan 4 bergerak sesuai dengan y axis
    servo3.write(sudut34);
    servo4.write(sudut34);

    // selisih perubahan yaw
    float selisihYaw = abs(yaw - yawBefore);

    // servo 5
    if (!servo5Back) {
      if (selisihYaw > threshold) { // kalo yaw berubah besar servo mengikuti 
        servo5.write(sudut5);
        counter = 0;
      }
      else if (counter > 9) { // yaw stabil
        lastStableTime = millis();
        servo5Back = true;
        counter = 0;
      }
      else counter++;
    }
    else {
      if (millis() - lastStableTime > 1000) { // yaw stabil 1 detik balik ke tengah
        servo5.write(90);
        servo5Back = false;
      }
    }

    yawBefore = yaw; // update nilai yaw
  }

  delay(10); // delay agar servo ga jitter
}
