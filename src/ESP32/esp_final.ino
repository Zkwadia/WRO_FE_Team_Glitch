#include <math.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define BNO08X_RESET -1
#define reset_pin 22
#define BNO08X_CS 10
#define BNO08X_INT 9
#define reset_imu 8           
#define FAST_MODE
const int outputA = 4; //D2  //2;
const int outputB = 5; //D3 //3; 

// Initialize variables
volatile long counter = 0;
int aLastState;

long heading = 0;
String output = "";
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#ifdef FAST_MODE
// frequency is somewhat variable, check with your own board or use the exact model we use
sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
long reportIntervalUs = 2000;
#else
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {  
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}


void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}


void initializeBNO08x() {
  const int max_retries = 5;
  int retry_count = 0;

  while (retry_count < max_retries) {
    // Try to initialize the sensor
    if (bno08x.begin_I2C()) {
      Serial.println("BNO08x Found!");
      setReports(reportType, reportIntervalUs);
      return;
    } else {
      Serial.println("Failed to find BNO08x, retrying");
      retry_count++;
      delay(1000); 
    }
  }

  // If all retries fail, print an error and continue
  Serial.println("Failed to initialize BNO08x after multiple attempts.");
}

void setup() {
  pinMode(outputA, INPUT_PULLUP);
  pinMode(outputB, INPUT_PULLUP);
  pinMode(reset_imu, INPUT);
  Serial.begin(115200);

  aLastState = digitalRead(outputA);

  attachInterrupt(digitalPinToInterrupt(outputA), encoderReadISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputB), encoderReadISR, CHANGE);

  while (!Serial) delay(10); 

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  while (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    //while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");


  setReports(reportType, reportIntervalUs);
  Serial.println("READY");
  // Serial.println("Reading events");
  delay(100);

}

int prev_counter = 0;
void encoderReadISR() {
  int aState = digitalRead(outputA);

  if (aState != aLastState) {
    if (digitalRead(outputB) != aState) {

      counter++;

    } else {
      counter--;
    }
  }
  aLastState = aState;
}


void loop() {
  // Check for incoming serial data
  if (Serial.available() > 0) {
    char data = Serial.read();
    // Reset counter if data is "1"
    if (data == '1') {
      counter = 0;
      Serial.println(counter);
      // Serial.flush();
    }

    if(data == 'R'){
      ESP.restart();
    }
  }

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        break;
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    static long last = 0;
    long now = micros();
    last = now;
    // Serial.println(counter);
  }
  if (ypr.yaw > 0){
    ypr.yaw = 360 - ypr.yaw;
  }
  else{
    ypr.yaw = 0 - ypr.yaw;
  }
  output = (String)(ypr.yaw) + " " + (String)counter;
  Serial.println(output);
}