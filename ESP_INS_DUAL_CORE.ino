#include "INS_COMMON.h"
#include "uNavAHRS.h"
#include "MPU9250.h"
#include "EEPROM.h"
#include "estimator_ekf.h"
#include "GPS.h"
#include "SparkFunBME280.h"
#include "INSCOMS.h"
#include "INSPARAMS.h"
#include "ESP_IO.h"
#include "car_control.h"

// Cryptography stuff
#include <Crypto.h>
#include <AES.h>

#define DEBUG 0

#define RELEASE 1
#define KEY_SET 0

#define ESP32 1
#define GPSM9N 1
#define GLOBAL_DT 0.0020
#define GLOBAL_DT_US 1e6*GLOBAL_DT
#define SENSOR_LOOP_TIME_US 2000

MPU9250 Imu(SPI,5);
BME280 Baro;
GPS gps;
GCS gcs;

uNavAHRS Filter;
estimator_ekf state_filter;

controller control;

// timers to measure performance
unsigned long tstart, tstop;
float start_alt = 0;


void do_calib()
{
  uint8_t EepromBuffer[49];

  for(int i=0;i<6;i++)
  {
    for(int j =0;j<40;j++)
    {
      gcs.Send_Calib_Command(i);
      delay(100);
    }
    Imu.calibrateAccel();
  }
  gcs.Send_Calib_Command(6);
  delay(1000);
  Imu.calibrateMag();
  float SB[12];
  Imu.getSB(SB);
  for(int i=0;i<12;i++)
  {
    memcpy(EepromBuffer+4*i,&SB[i],4);
  }
  EepromBuffer[48] = 1; // set to true: caliberated
  for (size_t i=0; i < sizeof(EepromBuffer); i++) {
    EEPROM.write(i,EepromBuffer[i]);
    EEPROM.commit();
  }
  gcs.Send_Calib_Command(7);
  while(1)
  {
    delay(1000);
  }
}

void get_calib()
{
  // EEPROM buffer and variables to load accel and mag bias 
  // and scale factors from CalibrateMPU9250.inoz
  EEPROM.begin(512);
  uint8_t EepromBuffer[49];
  for (size_t i=0; i < sizeof(EepromBuffer); i++) 
  {
    EepromBuffer[i] = EEPROM.read(i);
  }
  if(EepromBuffer[48]==0 or EepromBuffer[48]==255) // two most common values
  {
    do_calib();
  }
  float SB[12];
  for(int i=0;i<12;i++)
  {
    memcpy(&SB[i],EepromBuffer+4*i,4);
  }
  Imu.setSB(SB);
}


void save_gyro_bias()
{
  uint8_t EepromBuffer[12];
  float value;
  value = Imu.getGyroBiasX_rads();
  memcpy(EepromBuffer,&value,4);
  value = Imu.getGyroBiasY_rads();
  memcpy(EepromBuffer+4,&value,4);
  value = Imu.getGyroBiasZ_rads();
  memcpy(EepromBuffer+8,&value,4);
  for (size_t i=0; i < sizeof(EepromBuffer); i++) {
    EEPROM.write(49+i,EepromBuffer[i]);
    EEPROM.commit();
  }
}

void use_saved_gyro_bias()
{
  float gxb,gyb,gzb;
  uint8_t EepromBuffer[12];
  uint8_t count=0;
  for(size_t i=0;i<sizeof(EepromBuffer);i++)
  {
    EepromBuffer[i] = EEPROM.read(i+49);
    if(EepromBuffer[i]==255 or EepromBuffer[i]==0)
    {
      count++;
    }
  }
  if(count<6)
  {
    memcpy(&gxb,EepromBuffer,4);
    memcpy(&gyb,EepromBuffer+4,4);
    memcpy(&gzb,EepromBuffer+8,4);
    Imu.setGyroBiasX_rads(gxb);
    Imu.setGyroBiasY_rads(gyb);
    Imu.setGyroBiasZ_rads(gzb);
  }
  else
  {
    Imu.setGyroBiasX_rads(0);
    Imu.setGyroBiasY_rads(0);
    Imu.setGyroBiasZ_rads(0);
  }
}

void key_check()
{
  crypto_feed_watchdog();
  AES256 cipher;
  byte buf[16], key[32], eID[16], EepromBuffer[16];
  uint64_t chipID = ESP.getEfuseMac();
  for(int i=0;i < 2;i++)
  {
    memcpy(eID + 8*i, &chipID, 8);
  }
  for(int i=0;i < 4;i++)
  {
    memcpy(key + 8*i, &chipID, 8); // thanos meme: I used the chipID to encrypt the chipID
  }
  cipher.setKey(key, 32);
  cipher.encryptBlock(buf, eID);
  
  #if DEBUG
  for(int i=0;i<16;i++)
  {
    Serial.print(buf[i],HEX);Serial.print("||");
  }
  Serial.println();
  #endif
  
  #if KEY_SET
  for (size_t i=0; i < 16; i++) 
  {
    EEPROM.write(64+i,buf[i]);
    EEPROM.commit();
  }
  #else
  for (size_t i=0; i < 16; i++)
  {
    EepromBuffer[i] = EEPROM.read(64+i);
  }
  #if DEBUG
  for(int i=0;i<16;i++)
  {
    Serial.print(buf[i],HEX);Serial.print("||");Serial.println(EepromBuffer[i],HEX);
  }
  #endif
  if(memcmp(buf, EepromBuffer, 32) == 0)
  {
    exit(0);
  }
  #endif
}

static SemaphoreHandle_t  sema_v; 
void filter_layer(void *parameter);
void wifi_coms_layer(void *parameter);

void setup() 
{
  sema_v = xSemaphoreCreateMutex(); // need me a sem

  // serial to display data
  Serial.begin(115200);
  // start communication with IMU 
  IO_init();
  while(not Baro.beginSPI(15));
  Baro.readTempC();
  Baro.readFloatHumidity();
  start_alt = Baro.readFloatAltitudeMeters();
  start_alt = Baro.readFloatAltitudeMeters();
  start_alt = Baro.readFloatAltitudeMeters();
  int status = Imu.begin();
  get_calib();

  gps.initialize(); // initialize gps
  
  bool gyro_calib_done=false;
  Vector3f accel_start,accel_end;
  float diff = 1.0f;
  for(int i=0;i<10 && diff>0.05;i++)
  {
    accel_start = Vector3f(Imu.getAccelX_mss(),Imu.getAccelY_mss(),Imu.getAccelZ_mss());
    Imu.calibrateGyro();
    accel_end = Vector3f(Imu.getAccelX_mss(),Imu.getAccelY_mss(),Imu.getAccelZ_mss());
    diff = (accel_start - accel_end).length();
  }
  if(diff<0.05f)
  {
    save_gyro_bias();
  }
  else if(diff>=0.05f)
  {
    use_saved_gyro_bias();
  }
  while(!Filter.update(Imu.getGyroX_rads(),Imu.getGyroY_rads(),Imu.getGyroZ_rads(),Imu.getAccelX_mss(),Imu.getAccelY_mss(),Imu.getAccelZ_mss(),Imu.getMagX_uT(),Imu.getMagY_uT(),Imu.getMagZ_uT()))
  {
    Imu.readSensor();    
  }
  #if RELEASE
  key_check();
  #endif
  state_filter.init_mag(Imu.getMagX_uT(),Imu.getMagY_uT(),Imu.getMagZ_uT());
  state_filter.setTimingData(gps.delta_t*1000, gps.delta_t*1000, 50, 25, 100); //vel pos baro mag airspeed delays
  xTaskCreatePinnedToCore(filter_layer, "sensor_task", 3000, NULL, 1, NULL,  0); 
}

long i = 0;
byte MODE = MODE_STANDBY,message;
bool reset_system = false;
long max_time = 0, dummy_timer=0;
float inputs[8];
float posNED[3]={0,0,0}; //dummy variable used for dry-tests.


void loop() 
{
    tstart = micros();

    get_Inputs(inputs);    
    
    state_filter.run_filter(reset_system);
    reset_system = false;
    
    if(gcs.failsafe or inputs[5]>=0)
    { 
      if(inputs[4]<= -0.6)
      {
        MODE = MODE_STANDBY;
      }
      if(inputs[4]>-0.6 and inputs[4]<0.6)
      {
        MODE = MODE_MANUAL;
      }
      if(inputs[4]>=0.6)
      {
        MODE = MODE_PARTIAL;
      }
    }

    float _G[3] = {state_filter._ekf->angRate.x,state_filter._ekf->angRate.y, state_filter._ekf->angRate.z};
    float _A[3] = {state_filter._ekf->accel.x, state_filter._ekf->accel.y, state_filter._ekf->accel.z};

    control.feedback(state_filter.bodyVelPred, state_filter._ekf->gpssAcc, state_filter._ekf->GPSstatus);
    
    if(MODE == MODE_PARTIAL || MODE == MODE_MANUAL || MODE == MODE_STOP || MODE == MODE_STANDBY || MODE==MODE_CONTROL_CHECK)//manual modes
    {
      float dum[] = {0.0,0.0};//dummy      
      control.driver(dum, 1000, state_filter._ekf->GPSstatus>=3, state_filter.bodyVelPred, _G, _A, MODE, inputs);
    }
    else
    {
      float dum[] = {0.00,0.00};//dummy
      MODE = MODE_STANDBY;
      control.driver(dum, 1000, state_filter._ekf->GPSstatus>=3, state_filter.bodyVelPred, _G, _A, MODE, inputs);
    }
   
    set_Output(0,control.steer);
    set_Output(1,control.throttle);

    tstop = micros();
    if(max_time < tstop - tstart)
    {
      max_time = tstop - tstart; 
    }
    
    if(message == SET_ORIGIN_ID)
    {
      reset_system = true;
      max_time = 0;
    }
    if(max_time>GLOBAL_DT_US)
    {
      max_time=0;
    }
    delayMicroseconds(max(0, int(GLOBAL_DT_US - (micros()-tstart) ) )); // instead of while(micros()-tstart < dt), we use delay as our general calc time is < 50% to keep it cooler.
    // shift to the while based method when we are starting to reach calc limits.
}

void filter_layer(void * parameter)
{
  UBaseType_t uxHighWaterMark;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 2/portTICK_PERIOD_MS;
  xLastWakeTime = xTaskGetTickCount();
  pinMode(2,OUTPUT);
  bool led_state = false;
  digitalWrite(2,led_state);
  uint32_t time_stamp_ms = millis();
  uint64_t time_stamp_us = micros();
  uint64_t dt;
  Baro.readTempC(); //this is to ensure correct temp humidity compensation.
  Baro.readFloatHumidity();
  float alt = Baro.readFloatAltitudeMeters();
  if(alt<0) // because ESP32 is a little bitch and won't read baro properly sometimes.
  {
    ESP.restart();
  }
  led_state = !led_state;
  digitalWrite(2,led_state);
  dummy_timer = millis();
  for(;;)
  {
    dt = micros()-time_stamp_us;
    time_stamp_ms = millis();
    time_stamp_us = micros();
    // read the sensor
    Imu.readSensor();
    // the IMU update is treated as a sensor read because the actual stuff happens in the ekf, the imu is used mostly for boot-strapping and error checking
    Filter.update(Imu.getGyroX_rads(),Imu.getGyroY_rads(),Imu.getGyroZ_rads(),Imu.getAccelX_mss(),Imu.getAccelY_mss(),Imu.getAccelZ_mss(),Imu.getMagX_uT(),Imu.getMagY_uT(),Imu.getMagZ_uT());
    // order in which info is provided doesn't matter as long as you do it before calling "run_filter"
    float A[3], G[3], M[3],zeros[3]={0,0,0},AHRS[5]={Filter.getRoll_rad(),Filter.getPitch_rad(),Filter.getHeading_rad(), Filter.getGyroBiasX_rads(), Filter.getGyroBiasY_rads()};
    Imu.getSensorRead(A,G,M);
    gps.localizer();
    Baro.readTempC(); //this is to ensure correct temp humidity compensation.
    Baro.readFloatHumidity();
    alt = Baro.readFloatAltitudeMeters();

    #if DEBUG
    double input_lat = 28.20000000, input_lon = 77.800000;
    float fake_hAcc = 0.5;

    float fake_t = time_stamp_ms*1e-3;
    float input_hgt = start_alt + sinf(fake_t)*0.5 - fake_t;
    float VNED[3]   = {float(random(-100,100))/1000.0f,float(random(-100,100))/1000.0f,float(random(-100,100))/1000.0f};
    for(int i =0;i<3;i++)
    {
      posNED[i] += float(random(-50,50))*1e-3;
      posNED[i] = min(max(posNED[i],-3.0f),3.0f);
    }
    calcLLH(posNED, input_lat, input_lon, input_hgt,input_lat, input_lon, input_hgt);
    if(gps.tick)
    {
      Serial.println(state_filter._ekf->P[7][7]);
      dummy_timer = millis();
    }
    #endif

    xSemaphoreTake(sema_v, 0 );
    
    state_filter.setIMUData(A, G, GLOBAL_DT, time_stamp_ms, time_stamp_us);
    state_filter.setMagData(M, zeros, Filter.magUpdated_ );
    
    #if DEBUG
    state_filter.setGPSData(input_lat, input_lon, input_hgt, float(0.000), float(0.000), VNED,fake_hAcc, 1.0f, 0.3f, float(0.0), float(2.0), gps.delta_t, 3,1.0, gps.tick);
    #else
    state_filter.setGPSData(gps.latitude, gps.longitude, gps.hMSL, gps.gSpeed, gps.headMot, gps.VelNED, gps.hAcc, gps.vAcc, gps.sAcc, gps.magDec, gps.magAcc, gps.delta_t, gps.fixType,gps.pDOP, gps.tick);
    #endif
    gps.tick = false; //reset tick to be false after data has been used.
    
    state_filter.setAirData(0,alt, 1.0,GLOBAL_DT,true,false);
    state_filter.setAhrsData(AHRS);
    Filter.accel_comp[0] = state_filter.Accel_compensation.x;
    Filter.accel_comp[1] = state_filter.Accel_compensation.y;
    Filter.accel_comp[2] = state_filter.Accel_compensation.z;
    float controlData[7] = {inputs[0],inputs[2],control.steer,control.throttle,0,0,0};  
    gcs.Send_Data(state_filter._ekf->states, A, G, M, AHRS, gps.latitude, gps.longitude, gps.hMSL, gps.VelNED, gps.hAcc, gps.vAcc, gps.sAcc, gps.fixType, alt, gps.numSV,controlData,1.0, max_time); //also regulated at 10Hz

    message = gcs.check();
    if(message == VEL_ID)
    {
      gcs.get_velpos();
      state_filter.setOdomData(gcs.external_inputs.data,true);
    }
    else if(message == CALIB_ID)
    {
      do_calib();
    }
    MODE = gcs.mode;
    xSemaphoreGive(sema_v);

    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  
}
