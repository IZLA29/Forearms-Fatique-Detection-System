#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "EMGFilters.h"
#include "arduinoFFT.h"

const char* ssid = "ESP32_EMG_Sensor";
const char* password = "password123"; 

WiFiServer server(80);
WiFiClient client;

const int emgPin1 = A0; 
const int emgPin2 = A1; 

EMGFilters myFilter1;
EMGFilters myFilter2;
SAMPLE_FREQUENCY sampleRate = SAMPLE_FREQ_500HZ; 
NOTCH_FREQUENCY humFreq = NOTCH_FREQ_50HZ;       

// --- การตั้งค่าสำหรับ FFT ---
const uint16_t samples = 128; // ต้องเป็นเลขยกกำลัง 2
const double samplingFrequency = 500.0;
double vReal1[samples], vImag1[samples];
double vReal2[samples], vImag2[samples];
int sampleIndex = 0;

// สร้างออบเจกต์ FFT สำหรับเวอร์ชัน 2.x
ArduinoFFT<double> FFT1 = ArduinoFFT<double>(vReal1, vImag1, samples, samplingFrequency);
ArduinoFFT<double> FFT2 = ArduinoFFT<double>(vReal2, vImag2, samples, samplingFrequency);

unsigned long timeStamp; 
unsigned long timeBudget;

Adafruit_MPU6050 mpu;
bool mpuFound = false;

// --- ตัวแปรสำหรับจับเวลาอัปเดต MNF ทุกๆ 5 วินาที ---
unsigned long lastMNFUpdateTime = 0;
const unsigned long mnfUpdateInterval = 2500; // 5000 มิลลิวินาที = 5 วินาที
double current_mnf1 = 0.0;
double current_mnf2 = 0.0;
bool isFirstMNF = true; // เอาไว้เช็คให้คำนวณทันทีในครั้งแรกสุด

void setup() {
  Serial.begin(250000); 

  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: "); Serial.println(IP);

  server.begin(); 
  
  myFilter1.init(sampleRate, humFreq, true, true, true);
  myFilter2.init(sampleRate, humFreq, true, true, true);
  timeBudget = 1e6 / sampleRate; 

  if (!mpu.begin()) {
    mpuFound = false;
  } else {
    mpuFound = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
}

void loop() {
  timeStamp = micros(); 

  if (!client || !client.connected()) {
    client = server.available();
  }

  // อ่านค่าและกรองสัญญาณ EMG
  int val1 = analogRead(emgPin1);
  int dataAfterFilter1 = myFilter1.update(val1); 
  int env1 = sq(dataAfterFilter1); 

  int val2 = analogRead(emgPin2);
  int dataAfterFilter2 = myFilter2.update(val2);
  int env2 = sq(dataAfterFilter2); 

  // เก็บข้อมูลสวิง (AC) เข้าบัฟเฟอร์
  vReal1[sampleIndex] = (double)dataAfterFilter1;
  vImag1[sampleIndex] = 0.0;
  vReal2[sampleIndex] = (double)dataAfterFilter2;
  vImag2[sampleIndex] = 0.0;
  sampleIndex++;

  // ประมวลผลเมื่อครบ 128 ตัว
  if (sampleIndex >= samples) {
    // คำนวณ FFT 1
    FFT1.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT1.compute(FFTDirection::Forward);
    FFT1.complexToMagnitude();

    // คำนวณ FFT 2
    FFT2.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT2.compute(FFTDirection::Forward);
    FFT2.complexToMagnitude();

    // 1. คำนวณ Mean Frequency (MNF) เฉพาะตอนเริ่มต้น หรือ เมื่อครบ 5 วินาที
    if (isFirstMNF || millis() - lastMNFUpdateTime >= mnfUpdateInterval) {
      double temp_mnf1 = 0, temp_mnf2 = 0;
      double sumMag1 = 0, sumFreqMag1 = 0;
      double sumMag2 = 0, sumFreqMag2 = 0;
      
      // เราใช้ค่าครึ่งเดียวของสเปกตรัม (Nyquist) คือ 64 ค่า
      for(int i = 1; i <= 64; i++) {
          double freq = i * (samplingFrequency / samples);
          sumMag1 += vReal1[i];
          sumFreqMag1 += (freq * vReal1[i]);
          sumMag2 += vReal2[i];
          sumFreqMag2 += (freq * vReal2[i]);
      }
      if(sumMag1 > 0) temp_mnf1 = sumFreqMag1 / sumMag1;
      if(sumMag2 > 0) temp_mnf2 = sumFreqMag2 / sumMag2;

      // อัปเดตค่าที่จะส่งไปโชว์
      current_mnf1 = temp_mnf1;
      current_mnf2 = temp_mnf2;
      
      // รีเซ็ตตัวจับเวลาและสถานะครั้งแรก
      lastMNFUpdateTime = millis();
      isFirstMNF = false;
    }

    // อ่านค่า MPU6050
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    if (mpuFound) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      ax = a.acceleration.x; ay = a.acceleration.y; az = a.acceleration.z;
      gx = g.gyro.x;         gy = g.gyro.y;         gz = g.gyro.z;
    }

    // 2. แพ็กข้อมูล 138 ค่า ส่งไปที่ MATLAB
    if (client && client.connected()) {
      // 8 ค่าแรก: Amp1, Amp2, MPU(6)
      client.printf("%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", env1, env2, ax, ay, az, gx, gy, gz);
      
      // 64 ค่าถัดมา: Spectrum EMG1
      for(int i = 1; i <= 64; i++) {
        client.printf(",%.1f", vReal1[i]);
      }
      
      // 64 ค่าถัดมา: Spectrum EMG2
      for(int i = 1; i <= 64; i++) {
        client.printf(",%.1f", vReal2[i]);
      }
      
      // 2 ค่าสุดท้าย: ใช้ค่า current_mnf ที่จะอัปเดตแค่ทุกๆ 5 วิ
      client.printf(",%.2f,%.2f\n", current_mnf1, current_mnf2);
    }

    sampleIndex = 0;
  }

  // ควบคุม Sampling Rate ที่ 500Hz
  unsigned long timeCost = micros() - timeStamp; 
  if (timeCost < timeBudget) {
    delayMicroseconds(timeBudget - timeCost);
  }
}