#include <FIRFilter.h>
#include <IIRFilter.h>

#include <heartRate.h>
#include <MAX30105.h>
#include <spo2_algorithm.h>

#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

uint16_t leftIrBuffer[100]; //infrared LED sensor data
uint16_t leftRedBuffer[100];  //red LED sensor data

uint8_t bufferLength;       // data length
uint8_t leftSpO2;           // Left SpO2 value
uint8_t leftHeartRate;      // Left heart rate value
uint16_t leftECG;
uint16_t redBuffer;
uint16_t irBuffer;

byte buff[15];
byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

//SoftwareSerial FBL780(2,3); // RX, TX


//Filter 
const double ECG_samplefreq  = 360;
//const uint8_t ECG_pin = A0;
const int16_t DC_offset = 511;
// 50 Hz notch
const double b_notch[] = { 1.39972748302835,  -1.79945496605670, 1.39972748302835 };
// 35 Hz Butterworth low-pass
const double b_lp[] = { 0.00113722762905776, 0.00568613814528881, 0.0113722762905776,  0.0113722762905776,  0.00568613814528881, 0.00113722762905776 };
const double a_lp[] = { 1, -3.03124451613593, 3.92924380774061,  -2.65660499035499, 0.928185738776705, -0.133188755896548 };
// 0.3 Hz high-pass
const double b_hp[] = { 1, -1 };
const double a_hp[] = { 1, -0.995 };
FIRFilter notch(b_notch);
IIRFilter lp(b_lp, a_lp);
IIRFilter hp(b_hp, a_hp);

void convertByte(uint16_t data, byte *tempBuff)
{
  tempBuff[0] = data & 255;
  tempBuff[1] = (data >> 8)  & 255;
}

void measureECG(byte *tempBuff)
{
 if((digitalRead(5) == 1)||(digitalRead(6) == 1)){
    convertByte(0, tempBuff);
    Serial.println(0);
  }
  else{
    leftECG = analogRead(A0);
//    delay(10);
double filtered = notch.filter(lp.filter(hp.filter(leftECG - DC_offset)));
  leftECG = round(filtered) + DC_offset;
    convertByte(leftECG, tempBuff);
    Serial.println(leftECG);
//    Serial2.write(leftECG);
  }
//  Serial.print('/');
}

void setup()
{
  Serial.begin(9600);  //serial
  Serial2.begin(9600); // FBL780BC BLE Module  serial2. Pin 17(RX), Pin 16(TX)
//  FBL780.begin(9600);
  Wire.begin();
  SPI.begin ();
//  Serial.print("setup");
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);
  pinMode(5, INPUT); // Setup for leads off detection LO +
  pinMode(6, INPUT); // Setup for leads off detection LO -

  digitalWrite(SS, HIGH);
  SPI.setClockDivider(SPI_CLOCK_DIV16);

  // Initialize sensor
//  if (!particleSensor.begin(Wire, I2C_SPEED_FAST,0x57)) //Use default I2C port, 400kHz speed
//  {Serial.print("errormax30105");
//    while (1);
//  }
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

//  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
}

void loop()
{ 
  
  const static unsigned long ECG_interval  = round(1e6 / ECG_samplefreq);
  static unsigned long ECG_prevmicros = micros();
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
//  Serial.print("loop\n");
  digitalWrite(SS, LOW);

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
//    while (particleSensor.available() == false) //do we have new data?
//      particleSensor.check(); //Check the sensor for new data
//    leftRedBuffer[i] = particleSensor.getRed();
//    leftIrBuffer[i] = particleSensor.getIR();
    leftRedBuffer[i] = 0;
    leftIrBuffer[i] = 0;
//    particleSensor.nextSample(); //We're finished with this sample so move to next sample
  }

  //calculate heart rate and leftSpO2 after first 100 samples (first 4 seconds of samples)
//  maxim_heart_rate_and_oxygen_saturation(leftIrBuffer, bufferLength, leftRedBuffer, &leftSpO2);

  //Continuously taking samples from MAX30102.  Heart rate and leftSpO2 are calculated every 1 second
  while (1)
  { 
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      leftRedBuffer[i - 25] = leftRedBuffer[i];
      leftIrBuffer[i - 25] = leftIrBuffer[i];
    }
    
    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
        if (!Serial) {
    ECG_prevmicros = micros();
  } else if (micros() - ECG_prevmicros >= ECG_interval) {
      measureECG(buff + 0);
          ECG_prevmicros += ECG_interval;
  }
      
//      while (particleSensor.available() == false) //do we have new data?
//        particleSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

        if (!Serial) {
    ECG_prevmicros = micros();
  } else if (micros() - ECG_prevmicros >= ECG_interval) {
      measureECG(buff + 2);
                ECG_prevmicros += ECG_interval;
  }
//      redBuffer = particleSensor.getRed();
      redBuffer = 0;
      leftRedBuffer[i] = redBuffer;
      
        if (!Serial) {
    ECG_prevmicros = micros();
  } else if (micros() - ECG_prevmicros >= ECG_interval) {
      measureECG(buff + 4);
                ECG_prevmicros += ECG_interval;
  }
      
//      irBuffer = particleSensor.getIR();
      irBuffer = 0;
      leftIrBuffer[i] = irBuffer;
//      particleSensor.nextSample(); //We're finished with this sample so move to next sample

        if (!Serial) {
    ECG_prevmicros = micros();
  } else if (micros() - ECG_prevmicros >= ECG_interval) {
      measureECG(buff + 6);
                ECG_prevmicros += ECG_interval;
  }
      delay(2);
//
//      Serial.print(redBuffer);
//      Serial.print('/');
//      Serial.print(irBuffer);
//      Serial.print('/');
      
      convertByte(redBuffer, buff + 8);
      convertByte(irBuffer, buff + 10);
      // SPI Communication: Heart Rate
//      leftHeartRate = SPI.transfer (0);
      leftHeartRate=0;
//      Serial.print(leftHeartRate);
//      Serial.print('/');
//      Serial.print(leftSpO2);
//      Serial.print('\n');
      
      buff[12] = leftHeartRate;
      buff[13] = leftSpO2;
      buff[14] = '\r';
//      FBL780.write(buff, sizeof(buff));
    Serial2.write(buff, sizeof(buff)); 
//    Serial.print(buff, sizeof(buff));
//    Serial.print("end loop\n");
      delay(1);
    } 

    //After gathering 25 new samples recalculate HR and SpO2
    maxim_heart_rate_and_oxygen_saturation(leftIrBuffer, bufferLength, leftRedBuffer, &leftSpO2);
  }
}

