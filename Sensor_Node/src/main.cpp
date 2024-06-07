/*=================================================================================================================
  Global Variables
==================================================================================================================*/
#include <list>
#include <Arduino.h>
#include <string.h>

// quick config C3
#define CALIBRATION_TIME 1 // 3 // Default calibration time for MICS-4514 is 3 minutes
#define WARMUP_TIME 1000   // 900000 // Default warm-up time for MICS-VZ-89TE is 15 minutes (900000 ms)
#define temp_mod 1         // Default multiplier for packet sizes is 1
bool initSD = false;       // remove payloads and reset files
bool viewLogMode = false;  // view contents of log.txts
bool offAlert = false;     // manually switch off alert mode
bool onAlert = true;       // manually switch on alert mode
bool switchAlert = false;  // manually switch between modes every 20 packets
int switchCounter = 0;     // counter for switchAlert
int prevBatt = 100;        // store previous battery level to get smooth discharge

// redundant buffer for payloads
std::list<String> payloadBacklog;

// bypass SD in case it dies
bool sdWorks = false;
int numDataPackets = 0;
int payloadFileName = 1;
int payloadToBeTransmitted = 1;

// battery info
float batt = 0;
float voltage = 0;
float current = 0;
float power = 0;

// elevation code
String elevationCode = "";

// mode tracker
bool warmUpMode = true;            // on initial startup, always warm up
bool offlineMode = false;          // fill up payload
bool onlineMode = false;           // upload payload (while filling up new payload)
bool alertMode = false;            // decrease sampling interval to 30s
bool sleepCommandReceived = false; // buffer for sleep command
unsigned long modeTime = 0;        // allow changes from alert to default only after 60s
unsigned long lastConnect = 0;     // record last time node was connected to the mesh

// mesh tracker
bool meshActive = false;
uint32_t nodeID = 0;
std::list<String> currentNodes;
unsigned long timeChecked = 0;

// task tracker
bool initalAddTasks = false;
bool taskWarmUpEnabled = true; // on initial startup, always warm up
bool taskGetMeasurementsEnabled = false;
bool taskGetMeasurementsAlertEnabled = false;
bool taskGetMeasurementsAlertOngoing = false;
bool taskSendMessageEnabled = false;

// packets
String packet = "";
int packetCounter = 0;

// payloads
String stringReceivePayload = "";
String stringSendPayload = "";
bool ackedChecker = false; // check if payloadToBeTransmitted can be updated

// datetime
// 10 Mar 2024 15:00:00
int second = 0;
int minute = 0;
int hour = 15;
int day = 10;
int month = 3;
int year = 2024;

// latency checking
bool latencyChecked = false;
unsigned long timeSent = 0;
unsigned long timeReceived = 0;
int latency = 0;

/*=================================================================================================================
  SD Card Definitions and Functions
==================================================================================================================*/
const int chipSelect = 7; // 5 for WROOM // 7 (?) for C3 //ESP32 pin number where SS pin on SD Card Module is connected
#include <SdFat.h>
// for FAT32 SD Cards
// SdFat sd;
// SdFile myFile;

// for exFat SD Cards
SdExFat sd;
ExFile myFile;

void SDCardWrite(const char fileName[], const char entry[])
{
  // open the file for write at end like the Native SD library
  if (!myFile.open(fileName, O_RDWR | O_CREAT | O_AT_END))
  {
    String errorMess = "opening " + String(fileName) + " for write failed";
    Serial.println(errorMess);
    sdWorks = false;
    // abort();
    // sd.errorHalt(errorMess.c_str());
  }
  // if the file opened okay, write to it:
  // Serial.println("Writing to " + String(fileName));
  myFile.print(entry);

  // close the file:
  myFile.close();
  // Serial.println("Done writing to " + String(fileName));
  // Serial.println();
}

void SDCardAppend(const char fileName[], const char entry[])
{
  // open the file for write at end like the Native SD library
  if (!myFile.open(fileName, O_RDWR | O_CREAT | O_AT_END))
  {
    String errorMess = "opening " + String(fileName) + " for append failed";
    Serial.println(errorMess);
    sdWorks = false;
    // abort();
    // sd.errorHalt(errorMess.c_str());
  }
  // if the file opened okay, write to it:
  // Serial.println("Writing to " + String(fileName));
  myFile.println(entry);

  // close the file:
  myFile.close();
  // Serial.println("Done appending to " + String(fileName));
  // Serial.println();
}

String SDCardRead(const char fileName[])
{
  // open the file for reading:
  if (!myFile.open(fileName, O_READ))
  {
    String errorMess = "opening " + String(fileName) + " for read failed";
    Serial.println(errorMess);
    sdWorks = false;
    return errorMess;
    // abort();
    // sd.errorHalt(errorMess.c_str());
  }

  // read from the file until there's nothing else in it:
  String data = "";
  while (myFile.available())
  {
    data += (char)myFile.read();
  }
  // close the file:
  myFile.close();

  return data;
}

/*=================================================================================================================
  Timestamp Definitions and Functions
==================================================================================================================*/
#include <ESP32Time.h>
ESP32Time rtc(0); // GMT+8 offset in seconds, no need because python datetime is already adjusted

void updateTime()
{
  second = rtc.getSecond();
  minute = rtc.getMinute();
  hour = rtc.getHour(true);
  day = rtc.getDay();
  month = rtc.getMonth() + 1;
  year = rtc.getYear();
}

/*=================================================================================================================
  Log Definitions and Functions
==================================================================================================================*/

void log(String entry = " ")
{
  // SDCardWrite("log.txt", entry.c_str());
  Serial.print(entry);
}

void logln(String entry = " ")
{
  String entry2 = rtc.getDateTime() + "    " + entry;
  // SDCardAppend("log.txt", entry2.c_str());
  Serial.println(entry2);
}

/*=================================================================================================================
  Sensor Definitions and Functions
==================================================================================================================*/
#include <Wire.h>

// INA219
#include <Adafruit_INA219.h>
Adafruit_INA219 ina219;

/*
float getVoltage()
{
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;

  try
  {
    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    loadvoltage = busvoltage + (shuntvoltage / 1000);
  }
  catch (...)
  {
    loadvoltage = 0;
  }
  log("Battery Voltage = ");
  log(String(loadvoltage));
  logln(" %");
  return loadvoltage;
}

float getCurrent()
{
  float current_mA = 0;
  try
  {
    current_mA = ina219.getCurrent_mA();
  }
  catch (...)
  {
    current_mA = 0;
  }
  log("Battery Current = ");
  log(String(current_mA));
  logln(" mA");
  return current_mA;
}
*/

float getBatt()
{
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  float batt = 0;
  try
  {
    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);

    // compute battery percentage
    batt = (loadvoltage - 2.9) / (3.7 - 2.9) * 100;
    if (batt < 0)
    {
      batt = 0;
    }
  }
  catch (...)
  {
    batt = 0;
  }
  // update global battery info
  voltage = loadvoltage;
  current = current_mA;
  power = voltage * current;

  logln("Battery Level = " + String(batt) + " %");
  return batt;
}

// DFRobot BME680
// #include <DFRobot_BME680_I2C.h>
// DFRobot_BME680_I2C bme(0x76); // 0x77 Default, 0x76 accdg to DFRobot Wiki

// use a different libray for the C3
#include "bme680.h"
// #define BME680_DEBUG
struct bme680_dev gas_sensor;
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  int result = 0;

  Wire.beginTransmission(dev_id);
  Wire.write(reg_addr); // Accès au registre
  Wire.endTransmission(false);
  Wire.requestFrom(dev_id, len);
  if (len <= Wire.available())
  {
    for (int i = 0; i < len; i++)
    {
      data[i] = Wire.read();
    }
  }
  else
  {
    result = 5;
  }
  return result;
}
/**
 * Write the content into the register targeted register for a specified device
 *
 * Parameter :
 *    dev_id : the I2C adress of the device
 *    reg_addr : the adresse of the targeted register
 *    *data : pointer to the data to write
 *    len : number in byte of the data
 *
 * Return:  Error code
 */
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  int result = 0;
  Wire.beginTransmission(dev_id);
  Wire.write(reg_addr);
  for (int i = 0; i < len; i++)
  {
    Wire.write(data[i]);
  }
  Wire.endTransmission();
  return result;
}
/**
 * function to wait the time
 *
 * Parameter :
 *    period : time that will be waited in millisecond
 */
void user_delay_ms(uint32_t period)
{
  delay(period);
}

int8_t rslt = BME680_OK;
void init_bme(void)
{
  uint8_t set_required_settings;
  /* Set the temperature, pressure and humidity settings */
  gas_sensor.tph_sett.os_hum = BME680_OS_2X;
  gas_sensor.tph_sett.os_pres = BME680_OS_4X;
  gas_sensor.tph_sett.os_temp = BME680_OS_8X;
  gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;
  /* Set the remaining gas sensor settings and link the heating profile */
  gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
  /* Create a ramp heat waveform in 3 steps */
  gas_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
  gas_sensor.gas_sett.heatr_dur = 150;  /* milliseconds */
  /* Select the power mode */
  /* Must be set before writing the sensor configuration */
  gas_sensor.power_mode = BME680_FORCED_MODE;
  /* Set the required sensor settings needed */
  set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL | BME680_GAS_SENSOR_SEL;
  /* Set the desired sensor configuration */
  rslt = bme680_set_sensor_settings(set_required_settings, &gas_sensor);
  /* Set the power mode */
  rslt = bme680_set_sensor_mode(&gas_sensor);
}

void getBME680Values()
{
  rslt = bme680_init(&gas_sensor);
  init_bme();
}

float getRelativeHumidity()
{
  struct bme680_field_data data;
  rslt = bme680_get_sensor_data(&data, &gas_sensor);
  rslt = bme680_get_sensor_data(&data, &gas_sensor);
  rslt = bme680_get_sensor_data(&data, &gas_sensor);

  // read humidity
  float humidity = 0;
  try
  {
    humidity = data.humidity / 1000;
  }
  catch (...)
  {
    humidity = 0;
  }

  /* Trigger the next measurement if you would like to read data out continuously */
  if (gas_sensor.power_mode == BME680_FORCED_MODE)
  {
    rslt = bme680_set_sensor_mode(&gas_sensor);
  }

  logln("Humidity = " + String(humidity) + " %");
  return humidity;
}

float getTemperature()
{
  struct bme680_field_data data;
  rslt = bme680_get_sensor_data(&data, &gas_sensor);
  rslt = bme680_get_sensor_data(&data, &gas_sensor);
  rslt = bme680_get_sensor_data(&data, &gas_sensor);

  // read temperature
  float temperature = 0;
  try
  {
    temperature = (data.temperature / 100) - 5;
  }
  catch (...)
  {
    temperature = 0;
  }

  /* Trigger the next measurement if you would like to read data out continuously */
  if (gas_sensor.power_mode == BME680_FORCED_MODE)
  {
    rslt = bme680_set_sensor_mode(&gas_sensor);
  }

  logln("Temperature = " + String(temperature) + " *C");
  return temperature;
}

// DFRobot MiCS-4514
#include <DFRobot_MICS.h>
DFRobot_MICS_I2C mics(&Wire, 0x75); // 0x75 I2C address

float getCO()
{
  // read co
  float co = 0;
  try
  {
    co = mics.getGasData(CO);
  }
  catch (...)
  {
    co = 0;
  }

  logln("CO = " + String(co) + " ppm");
  return co;
}

float getNO2()
{
  // read no2
  float no2 = 0;

  try
  {
    no2 = mics.getGasData(NO2) * 1000;
  }
  catch (...)
  {
    no2 = 0;
  }
  logln("NO2 = " + String(no2) + " ppb");
  return no2;
}

// MiCS-VZ89TE
#include <MICS-VZ-89TE.h>
MICS_VZ_89TE micsVZ; // 0x70 I2C address

void getMICSVZValues()
{
  micsVZ.readSensor();
}

float getCO2()
{
  // read co2
  float co2 = 0;
  try
  {
    co2 = micsVZ.getCO2();
  }
  catch (...)
  {
    co2 = 0;
  }

  logln("CO2 = " + String(co2) + " ppm");
  return co2;
}

float getTVOC()
{
  // read tvoc
  float tvoc = 0;
  try
  {
    tvoc = micsVZ.getVOC();
  }
  catch (...)
  {
    tvoc = 0;
  }

  logln("TVOC = " + String(tvoc) + " ppb");
  return tvoc;
}

// ULPSMSO2 986-006
#define VGAS 1 // 36 for WROOM, 1 for C3
#define VREF 0 // 39 for WROOM, 0 for C3

float getSO2()
{
  // read so2
  float so2 = 0;
  try
  {
    float vref = (analogRead(VREF) * 3.3 / 4096);
    float voffset = 0;

    float vgas_0 = vref + voffset;
    float vgas = (analogRead(VGAS) * 3.3 / 4096);

    // Different Sens for each SO2 Sensor
    // 36.95 for Node 136918729
    // 45.84 for Node 136926501
    // 37.04 for Node 136915333
    // 35.75 for Node 136921897
    // (?) Default is Average = 38.895

    float sens = 38.985;
    switch (nodeID)
    {
    case 136918729:
      sens = 36.95;
      break;

    case 136926501:
      sens = 45.84;
      break;

    case 136915333:
      sens = 37.04;
      break;

    case 136921897:
      sens = 35.75;
      break;

    default:
      break;
    }

    // Different Gain for each SO2 Circuit Board
    // remove SO2 sensor and check sticker

    float gain = 100;
    float M = sens * gain * 10e-9 * 10e3;

    so2 = (1 / M) * (vgas - vgas_0) * 1000;

    if (so2 < 0)
    {
      so2 = 0;
    }

    /*
    // Zero Offset Correction
    if (temperature >= -20 && temperature < 0)
    {
      so2 += 0.012 * temperature;
    }
    else if (temperature >= 0 && temperature < 25)
    {
      so2 += 0.056 * temperature;
    }
    else if (temperature >= 25 && temperature <= 40)
    {
      so2 += 0.46 * temperature;
    }

    // Span Sensitivity Correction
    if (temperature >= -20 && temperature < 20)
    {
      so2 *= 1 + ((-0.33 / 100) * temperature);
    }
    else if (temperature >= 20 && temperature <= 40)
    {
      so2 *= 1 + ((0.26 / 100) * temperature);
    }
    */
  }
  catch (...)
  {
    so2 = 0;
  }
  logln("SO2 = " + String(so2) + " ppb");
  return so2;
}

// SPS30
#include <sps30.h>
#define SP30_COMMS Wire
#define DEBUG_ 0
// function prototypes (sometimes the pre-processor does not create prototypes themself on ESPxx)
void serialTrigger(char *mess);
void ErrtoMess(char *mess, uint8_t r);
void Errorloop(char *mess, uint8_t r);
void GetDeviceInfo();
// create constructor
SPS30 sps30;

void GetDeviceInfo()
{
  char buf[32];
  uint8_t ret;
  SPS30_version v;

  // try to read serial number
  ret = sps30.GetSerialNumber(buf, 32);
  if (ret == SPS30_ERR_OK)
  {
    log(F("Serial number : "));
    if (strlen(buf) > 0)
      logln(buf);
    else
      logln(F("not available"));
  }
  else
    ErrtoMess((char *)"could not get serial number. ", ret);

  // try to get product name
  ret = sps30.GetProductName(buf, 32);
  if (ret == SPS30_ERR_OK)
  {
    log(F("Product name  : "));

    if (strlen(buf) > 0)
      logln(buf);
    else
      logln(F("not available"));
  }
  else
    ErrtoMess((char *)"could not get product name. ", ret);

  // try to get version info
  ret = sps30.GetVersion(&v);
  if (ret != SPS30_ERR_OK)
  {
    logln(F("Can not read version info."));
    return;
  }

  log(F("Firmware level: "));
  log(v.major);
  log(".");
  log(v.minor);
  logln();

  log(F("Library level : "));
  log(v.DRV_major);
  log(".");
  log(v.DRV_minor);
  logln();
}

/**
 * @brief : read and display all values
 */
bool read_all()
{
  static bool header = true;
  uint8_t ret, error_cnt = 0;
  struct sps_values val;

  // loop to get data
  do
  {

#ifdef USE_50K_SPEED            // update 1.4.3
    SP30_COMMS.setClock(50000); // set to 50K
    ret = sps30.GetValues(&val);
    SP30_COMMS.setClock(100000); // reset to 100K in case other sensors are on the same I2C-channel
#else
    ret = sps30.GetValues(&val);
#endif

    // data might not have been ready
    if (ret == SPS30_ERR_DATALENGTH)
    {

      if (error_cnt++ > 3)
      {
        ErrtoMess((char *)"Error during reading values: ", ret);
        return (false);
      }
      delay(1000);
    }

    // if other error
    else if (ret != SPS30_ERR_OK)
    {
      ErrtoMess((char *)"Error during reading values: ", ret);
      return (false);
    }

  } while (ret != SPS30_ERR_OK);

  // only print header first time
  if (header)
  {
    logln(F("-------------Mass -----------    ------------- Number --------------   -Average-"));
    logln(F("     Concentration [μg/m3]             Concentration [#/cm3]             [μm]"));
    logln(F("P1.0\tP2.5\tP4.0\tP10\tP0.5\tP1.0\tP2.5\tP4.0\tP10\tPartSize\n"));
    header = false;
  }

  log(val.MassPM1);
  log(F("\t"));
  log(val.MassPM2);
  log(F("\t"));
  log(val.MassPM4);
  log(F("\t"));
  log(val.MassPM10);
  log(F("\t"));
  log(val.NumPM0);
  log(F("\t"));
  log(val.NumPM1);
  log(F("\t"));
  log(val.NumPM2);
  log(F("\t"));
  log(val.NumPM4);
  log(F("\t"));
  log(val.NumPM10);
  log(F("\t"));
  log(val.PartSize);
  log(F("\n"));

  return (true);
}

/**
 *  @brief : continued loop after fatal error
 *  @param mess : message to display
 *  @param r : error code
 *
 *  if r is zero, it will only display the message
 */
void Errorloop(char *mess, uint8_t r)
{
  if (r)
    ErrtoMess(mess, r);
  else
    logln(mess);
  logln(F("Program on hold"));
  for (;;)
    delay(100000);
}

/**
 *  @brief : display error message
 *  @param mess : message to display
 *  @param r : error code
 *
 */
void ErrtoMess(char *mess, uint8_t r)
{
  char buf[80];

  log(mess);

  sps30.GetErrDescription(r, buf, 80);
  logln(buf);
}

/**
 * serialTrigger prints repeated message, then waits for enter
 * to come in from the serial port.
 */
void serialTrigger(char *mess)
{
  logln();

  while (!Serial.available())
  {
    logln(mess);
    delay(2000);
  }

  while (Serial.available())
    Serial.read();
}

float getPM25()
{
  // read pm25
  float pm25 = 0;
  try
  {
    pm25 = sps30.GetMassPM2();
  }
  catch (...)
  {
    pm25 = 0;
  }

  logln("pm25 = " + String(pm25) + " ug/m^3");
  return pm25;
}

/*=================================================================================================================
  AQI Definitions and Functions
==================================================================================================================*/

float getAQI(float &humidity, float &temperature, float &co, float &no2, float &co2, float &tvoc, float &so2, float &pm25)
{
  // assume parameters are not elevated initially
  alertMode = false;
  elevationCode = "";

  float AQI = 0;
  float IHI = 0;
  float ILO = 0;
  float BPHI = 0;
  float BPLO = 0;

  // compute humidity AQI
  if (humidity >= 63 && humidity <= 83)
  {
    IHI = 50;
    ILO = 0;
    BPHI = 83;
    BPLO = 63;
  }
  else if (humidity > 83)
  {
    logln("Relative Humidity is elevated!");
    alertMode = true;
    elevationCode += "1";

    IHI = 75;
    ILO = 51;
    BPHI = 100;
    BPLO = 83;
  }
  else if (humidity < 63)
  {
    logln("Relative Humidity is elevated!");
    alertMode = true;
    elevationCode += "1";

    IHI = 75;
    ILO = 51;
    BPHI = 0;
    BPLO = 63;
  }

  try
  {
    float tempAQI = ((IHI - ILO) / (BPHI - BPLO)) * (humidity - BPLO) + ILO;
    if (tempAQI > AQI)
    {
      AQI = tempAQI;
    }
  }
  catch (...)
  {
    AQI = 0;
  }

  // compute temperature AQI
  if (temperature >= 27.5 && temperature <= 33.5)
  {
    IHI = 50;
    ILO = 0;
    BPHI = 33.5;
    BPLO = 27.5;
  }
  else if (temperature > 33.5)
  {
    logln("Temperature is elevated!");
    alertMode = true;
    elevationCode += "2";

    IHI = 150;
    ILO = 101;
    BPHI = 85;
    BPLO = 33.5;
  }
  else if (temperature < 27.5)
  {
    logln("Temperature is elevated!");
    alertMode = true;
    elevationCode += "2";

    IHI = 150;
    ILO = 101;
    BPHI = -45;
    BPLO = 27.5;
  }

  try
  {
    float tempAQI = ((IHI - ILO) / (BPHI - BPLO)) * (temperature - BPLO) + ILO;
    if (tempAQI > AQI)
    {
      AQI = tempAQI;
    }
  }
  catch (...)
  {
    AQI = 0;
  }

  // bypass elevated RH and temp for now
  alertMode = false;

  // compute co AQI
  if (co >= 0 && co <= 35)
  {
    IHI = 50;
    ILO = 0;
    BPHI = 35;
    BPLO = 0;
  }
  else if (co > 35 && co <= 50)
  {
    logln("Carbon Monoxide is elevated!");
    alertMode = true;
    elevationCode += "3";

    IHI = 75;
    ILO = 51;
    BPHI = 50;
    BPLO = 35;
  }
  else if (co > 50 && co <= 70)
  {
    logln("Carbon Monoxide is elevated!");
    alertMode = true;
    elevationCode += "3";

    IHI = 100;
    ILO = 76;
    BPHI = 70;
    BPLO = 50;
  }
  else if (co > 70)
  {
    logln("Carbon Monoxide is elevated!");
    alertMode = true;
    elevationCode += "3";

    IHI = 150;
    ILO = 101;
    BPHI = 1000;
    BPLO = 70;
  }

  try
  {
    float tempAQI = ((IHI - ILO) / (BPHI - BPLO)) * (co - BPLO) + ILO;
    if (tempAQI > AQI)
    {
      AQI = tempAQI;
    }
  }
  catch (...)
  {
    AQI = 0;
  }

  // compute no2 AQI
  if (no2 >= 0 && no2 <= 100)
  {
    IHI = 50;
    ILO = 0;
    BPHI = 100;
    BPLO = 0;
  }
  else if (no2 > 100 && no2 <= 175)
  {
    logln("Nitrogen Dioxide is elevated!");
    alertMode = true;
    elevationCode += "4";

    IHI = 75;
    ILO = 51;
    BPHI = 175;
    BPLO = 100;
  }
  else if (no2 > 175 && no2 <= 250)
  {
    logln("Nitrogen Dioxide is elevated!");
    alertMode = true;
    elevationCode += "4";

    IHI = 100;
    ILO = 76;
    BPHI = 250;
    BPLO = 175;
  }
  else if (no2 > 250)
  {
    logln("Nitrogen Dioxide is elevated!");
    alertMode = true;
    elevationCode += "4";

    IHI = 150;
    ILO = 101;
    BPHI = 10000;
    BPLO = 250;
  }

  try
  {
    float tempAQI = ((IHI - ILO) / (BPHI - BPLO)) * (no2 - BPLO) + ILO;
    if (tempAQI > AQI)
    {
      AQI = tempAQI;
    }
  }
  catch (...)
  {
    AQI = 0;
  }

  // compute co2 AQI
  if (co2 >= 0 && co2 <= 800)
  {
    IHI = 50;
    ILO = 0;
    BPHI = 800;
    BPLO = 0;
  }
  else if (co2 > 800 && co2 <= 1150)
  {
    logln("Carbon Dioxide is elevated!");
    alertMode = true;
    elevationCode += "5";

    IHI = 75;
    ILO = 51;
    BPHI = 1150;
    BPLO = 800;
  }
  else if (co2 > 1150 && co2 <= 1500)
  {
    logln("Carbon Dioxide is elevated!");
    alertMode = true;
    elevationCode += "5";

    IHI = 100;
    ILO = 76;
    BPHI = 1500;
    BPLO = 1150;
  }
  else if (co2 > 1500)
  {
    logln("Carbon Dioxide is elevated!");
    alertMode = true;
    elevationCode += "5";

    IHI = 150;
    ILO = 101;
    BPHI = 2000;
    BPLO = 1500;
  }

  try
  {
    float tempAQI = ((IHI - ILO) / (BPHI - BPLO)) * (co2 - BPLO) + ILO;
    if (tempAQI > AQI)
    {
      AQI = tempAQI;
    }
  }
  catch (...)
  {
    AQI = 0;
  }

  // compute tvoc AQI
  if (tvoc >= 0 && tvoc <= 400)
  {
    IHI = 50;
    ILO = 0;
    BPHI = 400;
    BPLO = 0;
  }
  else if (tvoc > 400 && tvoc <= 600)
  {
    logln("TVOCs are elevated!");
    alertMode = true;
    elevationCode += "6";

    IHI = 75;
    ILO = 51;
    BPHI = 600;
    BPLO = 400;
  }
  else if (tvoc > 600 && tvoc <= 800)
  {
    logln("TVOCs are elevated!");
    alertMode = true;
    elevationCode += "6";

    IHI = 100;
    ILO = 76;
    BPHI = 800;
    BPLO = 600;
  }
  else if (tvoc > 800)
  {
    logln("TVOCs are elevated!");
    alertMode = true;
    elevationCode += "6";

    IHI = 150;
    ILO = 101;
    BPHI = 1000;
    BPLO = 800;
  }

  try
  {
    float tempAQI = ((IHI - ILO) / (BPHI - BPLO)) * (tvoc - BPLO) + ILO;
    if (tempAQI > AQI)
    {
      AQI = tempAQI;
    }
  }
  catch (...)
  {
    AQI = 0;
  }

  // compute so2 AQI
  if (so2 >= 0 && so2 <= 500)
  {
    IHI = 50;
    ILO = 0;
    BPHI = 500;
    BPLO = 0;
  }
  else if (so2 > 500 && so2 <= 750)
  {
    logln("Sulfur Dioxide is elevated!");
    alertMode = true;
    elevationCode += "7";

    IHI = 75;
    ILO = 51;
    BPHI = 750;
    BPLO = 500;
  }
  else if (so2 > 750 && so2 <= 1000)
  {
    logln("Sulfur Dioxide is elevated!");
    alertMode = true;
    elevationCode += "7";

    IHI = 100;
    ILO = 76;
    BPHI = 1000;
    BPLO = 750;
  }
  else if (so2 > 1000)
  {
    logln("Sulfur Dioxide is elevated!");
    alertMode = true;
    elevationCode += "7";

    IHI = 150;
    ILO = 101;
    BPHI = 20000;
    BPLO = 1000;
  }

  try
  {
    float tempAQI = ((IHI - ILO) / (BPHI - BPLO)) * (so2 - BPLO) + ILO;
    if (tempAQI > AQI)
    {
      AQI = tempAQI;
    }
  }
  catch (...)
  {
    AQI = 0;
  }

  // compute pm25 AQI
  if (pm25 >= 0 && pm25 <= 15)
  {
    IHI = 50;
    ILO = 0;
    BPHI = 15;
    BPLO = 0;
  }
  else if (pm25 > 15 && pm25 <= 20)
  {
    logln("PM2.5 is elevated!");
    alertMode = true;
    elevationCode += "8";

    IHI = 75;
    ILO = 51;
    BPHI = 20;
    BPLO = 15;
  }
  else if (pm25 > 20 && pm25 <= 30)
  {
    logln("PM2.5 is elevated!");
    alertMode = true;
    elevationCode += "8";

    IHI = 100;
    ILO = 76;
    BPHI = 30;
    BPLO = 20;
  }
  else if (pm25 > 30)
  {
    logln("PM2.5 is elevated!");
    alertMode = true;
    elevationCode += "8";

    IHI = 150;
    ILO = 101;
    BPHI = 1000;
    BPLO = 30;
  }

  try
  {
    float tempAQI = ((IHI - ILO) / (BPHI - BPLO)) * (pm25 - BPLO) + ILO;
    if (tempAQI > AQI)
    {
      AQI = tempAQI;
    }
  }
  catch (...)
  {
    AQI = 0;
  }
  // for testing modes manually
  if (offAlert)
  {
    alertMode = false;
  }

  if (onAlert)
  {
    alertMode = true;
  }

  logln("AQI = " + String(AQI));
  return AQI;
}

/*=================================================================================================================
  Task Scheduler Definitions and Functions
==================================================================================================================*/
#include <painlessMesh.h>
#include <WiFi.h>

Scheduler userScheduler;
painlessMesh mesh;

void warmUp();
Task taskWarmUp(TASK_SECOND * 1, TASK_FOREVER, &warmUp);
void warmUp()
{
  // On initial wake-up, warm-up MICS-4514 and MICS-VZ-89TE
  if (!mics.warmUpTime(CALIBRATION_TIME)) // 3 minutes CALIBRATION_TIME
  {
    logln("MICS-4514 has warmed up for " + String(millis() / 1000.0 / 60.0) + " out of 3 minutes");
  }
  else if (millis() < WARMUP_TIME) // 900000 = 15 minutes
  {
    logln("MICS-VZ-89TE has warmed up for " + String(millis() / 1000.0 / 60.0) + " out of 15 minutes");
  }
  else
  {
    logln("Warm-up done!");
    taskWarmUp.disable();
    warmUpMode = false;
    taskGetMeasurementsAlertEnabled = true;
  }
}

void getMeasurements(); // Prototype so PlatformIO doesn't complain
Task taskGetMeasurements(TASK_SECOND * 60, TASK_FOREVER, &getMeasurements);
Task taskGetMeasurementsAlert(TASK_SECOND * 60, TASK_FOREVER, &getMeasurements); // for alert mode // 60 for PGH
void getMeasurements()
{
  logln("Measuring...");

  // re-initialize some sensors
  getBME680Values();
  getMICSVZValues();

  // update switchAlert packet counter
  if (switchAlert && switchCounter >= 20)
  {
    offAlert = !offAlert;
    onAlert = !onAlert;
    switchCounter = 0;
  }
  switchCounter += 1;

  // update number of data packets
  if (sdWorks)
  {
    numDataPackets = SDCardRead("numDataPackets.txt").toInt(); // get number of data packets as integer
  }
  numDataPackets += 1;                                           // increment number of data packets
  String stringNumDataPackets = String(numDataPackets);          // convert from integer to String
  const char *charNumDataPackets = stringNumDataPackets.c_str(); // convert number of data packets from int to String to char
  sd.remove("numDataPackets.txt");                               // overwrite contents
  SDCardWrite("numDataPackets.txt", charNumDataPackets);
  logln("Number of total data packets = " + numDataPackets);
  logln();

  // construct packet
  updateTime();

  /*
  // ISO time stamp example: "2023-03-19T18:03:34"
  String month = "";
  if (rtc.getMonth() + 1 < 10)
  {
    month = String(0) + String(rtc.getMonth() + 1);
  }
  else
  {
    month = String(rtc.getMonth() + 1);
  }
  String date = "";
  if (rtc.getDay() < 10)
  {
    date = String(0) + String(rtc.getDay());
  }
  else
  {
    date = String(rtc.getDay());
  }
  String timeStamp = String(rtc.getYear()) + "-" + month + "-" + date + "T" + String(rtc.getTime());
  */

  String timeStamp = String(rtc.getEpoch());
  packet = "";
  packet = timeStamp + ",";

  // compute AQI from the parameters
  float humidity = getRelativeHumidity();
  float temperature = getTemperature();
  float co = getCO();
  float no2 = getNO2();
  float co2 = getCO2();
  float tvoc = getTVOC();
  float so2 = getSO2();
  float pm25 = getPM25();
  float aqi = getAQI(humidity, temperature, co, no2, co2, tvoc, so2, pm25);

  // get battery information
  batt = getBatt();
  // smooth out the battery discharge graph with 10 tries
  int counter = 0;
  while ((abs(prevBatt - batt) > 5) && counter < 10)
  {
    batt = getBatt();
    counter += 1;
  }
  if (abs(prevBatt - batt) > 5)
  {
    batt = prevBatt;
  }
  prevBatt = batt;

  // packet building
  packet += String(humidity, 2);
  packet += ",";
  packet += String(temperature, 2);
  packet += ",";
  packet += String(co, 2);
  packet += ",";
  packet += String(no2, 2);
  packet += ",";
  packet += String(co2, 2);
  packet += ",";
  packet += String(tvoc, 2);
  packet += ",";
  packet += String(so2, 2);
  packet += ",";
  packet += String(pm25, 2);
  packet += ",";
  packet += String(int(aqi));
  packet += ",";
  packet += String(int(batt));
  packet += ",";
  packet += String(voltage, 2);
  packet += ",";
  packet += String(current, 2);
  packet += ",";
  packet += String(power, 2);
  packet += ",";
  packet += String(numDataPackets);
  packet += ",";
  packet += elevationCode;
  packet += "/";

  logln("Packet = " + packet);

  // setup filenames
  if (sdWorks)
  {
    payloadFileName = SDCardRead("payloadFileName.txt").toInt(); // get number of data packets as integer
  }
  String StringFileName = String(payloadFileName);   // get payload file name without ".txt"
  StringFileName += ".txt";                          // add ".txt"
  const char *charFileName = StringFileName.c_str(); // convert string filename number to const char
  const char *charPacket = packet.c_str();           // convert string packet to const char

  // check alertMode
  if (!alertMode)
  {
    /*
    // disable alertMode
    taskGetMeasurementsAlertEnabled = false;
    taskGetMeasurementsAlertOngoing = false;
    taskGetMeasurementsAlert.disable();
    logln("taskGetMeasurementsAlert was disabled");

    // return to onlineMode and re-enable taskGetMeasurements after 60s
    offlineMode = false;
    taskGetMeasurementsEnabled = true;
    taskSendMessageEnabled = true;
    onlineMode = true;

    // measure file size of payload and new packet
    int payloadSize = SDCardRead(charFileName).length();
    int packetSize = packet.length();

    // in onlineMode, if current payload size + packet size is less than 75% of 1350, turn off mesh
    // and payloadToBeTransmitted has caught up
    // check untransmitted data
    int payloadToBeTransmitted = SDCardRead("payloadToBeTransmitted.txt").toInt(); // get file name of payload to be transmitted as integer
    int payloadFileName = SDCardRead("payloadFileName.txt").toInt();               // get file name of current payload as integer

    if (onlineMode && ((payloadSize + packetSize) * temp_mod < 0.75 * 1350) && (payloadToBeTransmitted == payloadFileName)) // temp mod of * to speed up tests
    {
      offlineMode = true;
      taskGetMeasurements.disable();
      onlineMode = false;
    }

    // in offlineMode, if current payload size + packet size exceeds 75% of 1350, start connecting
    if (offlineMode && ((payloadSize + packetSize) * temp_mod >= 0.75 * 1350)) // temp mod of * to speed up tests
    {
      offlineMode = false;
      taskGetMeasurementsEnabled = true;
      onlineMode = true;
    }

    // if current payload size + packet size will exceed 1350, store packet somewhere else
    // and start sending message
    if ((payloadSize + packetSize) * temp_mod >= 1350) // temp mod of * to speed up tests
    {
      // broadcast payload by enabling taskSendMessage in main loop
      if (!taskSendMessageEnabled)
      {
        taskSendMessageEnabled = true;
      }
      logln(StringFileName + " is full and ready to upload. Creating new payload...");

      // update payloadFileName.txt based on numDataPackets.txt
      numDataPackets = SDCardRead("numDataPackets.txt").toInt();     // get number of data packets as integer
      stringNumDataPackets = String(numDataPackets);                 // convert from integer to String
      const char *charNumDataPackets = stringNumDataPackets.c_str(); // convert from String to char
      sd.remove("payloadFileName.txt");                              // overwrite contents
      SDCardWrite("payloadFileName.txt", charNumDataPackets);

      // re-setup filenames
      StringFileName = SDCardRead("payloadFileName.txt"); // get payload file name without ".txt"
      StringFileName += ".txt";                           // add ".txt"
      charFileName = StringFileName.c_str();              // convert string filename number to const char

      // append node ID to latest payload if it doesn't exist
      if (!sd.exists(charFileName))
      {
        SDCardWrite(charFileName, (String(nodeID) + "/").c_str());
      }
    }
    // append packet
    SDCardWrite(charFileName, charPacket);
    logln();

    // check payload
    logln("Contents of " + String(charFileName) + ": ");
    logln(SDCardRead(charFileName));
    logln();
    logln("Size of " + String(charFileName) + ": ");
    logln(String(SDCardRead(charFileName).length() * temp_mod) + " bytes");
    logln();
    */
  }
  else // alertMode
  {
    // change modes
    offlineMode = false;
    onlineMode = true;

    // enable tasks for alertMode
    if (!taskGetMeasurementsAlertEnabled)
    {
      taskGetMeasurementsAlertEnabled = true;
    }
    // broadcast payload by enabling taskSendMessage in main loop
    if (!taskSendMessageEnabled)
    {
      taskSendMessageEnabled = true;
    }

    logln(StringFileName + " is ready to upload. Creating new payload...");

    // update payloadFileName.txt based on numDataPackets.txt
    if (sdWorks)
    {
      numDataPackets = SDCardRead("numDataPackets.txt").toInt(); // get number of data packets as integer
    }
    stringNumDataPackets = String(numDataPackets);                 // convert from integer to String
    const char *charNumDataPackets = stringNumDataPackets.c_str(); // convert from String to char
    sd.remove("payloadFileName.txt");                              // overwrite contents
    SDCardWrite("payloadFileName.txt", charNumDataPackets);

    // re-setup filenames
    if (sdWorks)
    {
      payloadFileName = SDCardRead("payloadFileName.txt").toInt(); // get number of data packets as integer
    }
    String StringFileName = String(payloadFileName); // get payload file name without ".txt"
    StringFileName += ".txt";                        // add ".txt"
    charFileName = StringFileName.c_str();           // convert string filename number to const char

    // append node ID to latest payload if it doesn't exist
    if (!sd.exists(charFileName))
    {
      SDCardWrite(charFileName, (String(nodeID) + "/").c_str());
    }

    // append packet to buffer
    stringSendPayload = String(mesh.getNodeId()) + "/" + packet;
    payloadBacklog.push_front(stringSendPayload);

    // back-up packet
    SDCardWrite(charFileName, charPacket);

    logln();

    // check payload
    logln("Contents of " + String(charFileName) + ": ");
    logln(SDCardRead(charFileName));
    logln("Backup: " + stringSendPayload);
    logln();
    logln("Size of " + String(charFileName) + ": ");
    logln(String(SDCardRead(charFileName).length() * temp_mod) + " bytes");
    logln("Size of Backup: " + stringSendPayload.length());
    logln();
  }

  // check files
  logln("numDataPackets.txt: " + SDCardRead("numDataPackets.txt"));
  logln("payloadFileName.txt: " + SDCardRead("payloadFileName.txt"));
  logln("payloadToBeTransmitted.txt: " + SDCardRead("payloadToBeTransmitted.txt"));

  /*
  // check untransmitted data
  if (sdWorks)
  {
    int payloadToBeTransmitted = SDCardRead("payloadToBeTransmitted.txt").toInt(); // get file name of payload to be transmitted as integer
    int payloadFileName = SDCardRead("payloadFileName.txt").toInt();               // get file name of current payload as integer
  }

  // check if sleep command was received
  // execute sleep only if not in alert mode

  if (!alertMode && sleepCommandReceived && (payloadToBeTransmitted == payloadFileName))
  {
    // change modes
    sleepCommandReceived = false;
    offlineMode = true;
    onlineMode = false;

    // reset payloads
    stringReceivePayload = "";

    // reset latency check
    latencyChecked = false;

    logln("Sleep command executed!");
    logln();
  }
  */

  // check node status
  if (onlineMode)
  {
    logln("onlineMode");
  }
  else if (offlineMode)
  {
    logln("offlineMode");
  }
  if (alertMode)
  {
    logln("alertMode");
  }
}

/*=================================================================================================================
  Mesh Definitions and Functions
==================================================================================================================*/
#define MESH_PREFIX "UP_CARE_TEAM_1I"
#define MESH_PASSWORD "UP_CARE_TEAM_1I"
#define MESH_PORT 5555

// function for sending message
void sendMessage(); // Prototype so PlatformIO doesn't complain
Task taskSendMessage(TASK_SECOND * 5, TASK_FOREVER, &sendMessage);
void sendMessage()
{
  /*
  if (sdWorks)
  {
    // taskGetMeasurements.disable();
    String payload = "";

    // only send latency messages when:
    //  - the payloadToBeTransmitted is an old payload and not the current
    //  - the file size is > 50 bytes
    // check untransmitted data
    int payloadToBeTransmitted = SDCardRead("payloadToBeTransmitted.txt").toInt(); // get file name of payload to be transmitted as integer
    int payloadFileName = SDCardRead("payloadFileName.txt").toInt();               // get file name of payload to be transmitted as integer

    logln("Checking if " + String(payloadToBeTransmitted) + ".txt exists...");
    if (sd.exists(String(payloadToBeTransmitted) + ".txt"))
    {
      logln(String(payloadToBeTransmitted) + ".txt exists!");
      // payload = SDCardRead((String(payloadToBeTransmitted) + ".txt").c_str());
      payload = payloadBacklog.front();

      // filter corrupted payloads
      if (payload.length() < 50 || payload[0] != '2')
      {
        latencyChecked = false;

        // delete corrupted payloads
        sd.remove(String(payloadToBeTransmitted) + ".txt");

        // update payloadToBeTransmitted.txt
        payloadBacklog.pop_front();
        stringSendPayload = payloadBacklog.front();

        payloadToBeTransmitted += 1;                                                   // increment filename
        String stringPayloadToBeTransmitted = String(payloadToBeTransmitted);          // convert from integer to String
        const char *charPayloadToBeTransmitted = stringPayloadToBeTransmitted.c_str(); // convert number of data packets from int to String to char
        sd.remove("payloadToBeTransmitted.txt");                                       // overwrite contents
        SDCardWrite("payloadToBeTransmitted.txt", charPayloadToBeTransmitted);
        logln("Payload to be transmitted = " + SDCardRead("payloadToBeTransmitted.txt"));
        logln();
        return;
      }
    }
    else
    {
      // this is causing some payloads to be skipped
      if (payloadToBeTransmitted < payloadFileName)
      {
        latencyChecked = false;

        // update payloadToBeTransmitted.txt
        payloadToBeTransmitted += 1;                                                   // increment filename
        String stringPayloadToBeTransmitted = String(payloadToBeTransmitted);          // convert from integer to String
        const char *charPayloadToBeTransmitted = stringPayloadToBeTransmitted.c_str(); // convert number of data packets from int to String to char
        sd.remove("payloadToBeTransmitted.txt");                                       // overwrite contents
        SDCardWrite("payloadToBeTransmitted.txt", charPayloadToBeTransmitted);
        logln("Payload to be transmitted = " + SDCardRead("payloadToBeTransmitted.txt"));
        logln();

        // update payloadToBeTransmitted from backlog
        stringSendPayload = payloadBacklog.front();
      }
      return;
    }
    // if (payloadToBeTransmitted != payloadFileName)
    if (!payloadBacklog.empty())
    {
      if (!latencyChecked)
      {
        logln("Sending latency message for " + String(payloadToBeTransmitted) + ".txt");
        mesh.sendBroadcast("l" + payload);
        timeSent = millis();
      }
      else
      {
        logln("Sending message for " + String(payloadToBeTransmitted) + ".txt");
        mesh.sendBroadcast(payload);
      }
    }
    else
    {
      taskGetMeasurementsAlertEnabled = true;
    }
  }
  */

  // don't send from the SD card

  // if (payloadToBeTransmitted != payloadFileName)
  if (!payloadBacklog.empty())
  {
    stringSendPayload = payloadBacklog.front();

    // filter corrupted payloads
    if (stringSendPayload.length() < 50 || stringSendPayload[0] != '2')
    {
      payloadBacklog.pop_front();
      stringSendPayload = payloadBacklog.front();
      return;
    }

    if (!latencyChecked)
    {
      logln("Sending latency message for " + String(payloadToBeTransmitted) + ".txt");
      mesh.sendBroadcast("l" + stringSendPayload);
      timeSent = millis();
    }
    else
    {
      logln("Sending message for " + String(payloadToBeTransmitted) + ".txt");
      mesh.sendBroadcast(stringSendPayload);
    }
  }

  // report payloadBacklog status
  logln("Current payloads in backlog = ");
  for (auto v : payloadBacklog)
  {
    log(v);
    log(", ");
  }
  logln();
  logln();
}

// function for processing received broadcasts
void receivedCallback(uint32_t from, String &msg)
{
  // logf("startHere: Received from %u msg=%s\n", from, msg.c_str());
  stringReceivePayload = msg.c_str();
  logln("Message from Mesh: ");
  logln(stringReceivePayload);
  logln();

  // check for timestamp data
  if (stringReceivePayload[0] == 't')
  {
    String receivedTime = stringReceivePayload.substring(1, stringReceivePayload.length());
    rtc.setTime(receivedTime.toInt());
    logln("Gateway Time = " + rtc.getDateTime());
  }

  // check for latency checking
  if (stringReceivePayload == "l" + String(mesh.getNodeId()) && !latencyChecked)
  {
    // get latency
    timeReceived = millis();
    latency = timeReceived - timeSent;

    if (payloadBacklog.empty())
    {
      return;
    }

    // retrieve packets from memory and append latency and RSSI
    String latencyAndRSSI = String(latency) + "," + String(WiFi.RSSI());
    SDCardWrite((SDCardRead("payloadToBeTransmitted.txt") + ".txt").c_str(), latencyAndRSSI.c_str());

    stringSendPayload = payloadBacklog.front();
    stringSendPayload += latencyAndRSSI;

    payloadBacklog.pop_front();
    payloadBacklog.push_front(stringSendPayload);

    String payload = SDCardRead((SDCardRead("payloadToBeTransmitted.txt") + ".txt").c_str());
    logln("Payload = " + payload);
    logln("Backup Payload = " + stringSendPayload);

    latencyChecked = true;
  }

  // check for ack that disables sendMessage
  if (stringReceivePayload == String(mesh.getNodeId()))
  {
    // update backup payload
    payloadBacklog.pop_front();
    stringSendPayload = payloadBacklog.front();

    latencyChecked = false;

    // store transmitted data and remove payload
    if (sd.exists((SDCardRead("payloadToBeTransmitted.txt") + ".txt")))
    {
      SDCardAppend("data.txt", SDCardRead((SDCardRead("payloadToBeTransmitted.txt") + ".txt").c_str()).c_str());
      sd.remove(SDCardRead("payloadToBeTransmitted.txt") + ".txt");
    }

    // check untransmitted data
    if (sdWorks)
    {
      int payloadToBeTransmitted = SDCardRead("payloadToBeTransmitted.txt").toInt(); // get file name of payload to be transmitted as integer
      int payloadFileName = SDCardRead("payloadFileName.txt").toInt();               // get file name of payload to be transmitted as integer
    }

    // if (payloadToBeTransmitted < payloadFileName)
    if (!payloadBacklog.empty())
    {
      // let sendMessage handle updating this
      /*
      latencyChecked = false;

      // update payloadToBeTransmitted.txt
      payloadToBeTransmitted += 1;                                                   // increment filename
      String stringPayloadToBeTransmitted = String(payloadToBeTransmitted);          // convert from integer to String
      const char *charPayloadToBeTransmitted = stringPayloadToBeTransmitted.c_str(); // convert number of data packets from int to String to char
      sd.remove("payloadToBeTransmitted.txt");                                       // overwrite contents
      SDCardWrite("payloadToBeTransmitted.txt", charPayloadToBeTransmitted);
      logln("Payload to be transmitted = " + SDCardRead("payloadToBeTransmitted.txt"));
      logln();
      */
    }
    else
    {
      // if we have transmitted all files before payloadFileName (which should be filling up),
      // if we have emptied the payload backlog buffer
      // then disable sending messages
      logln("taskSendMessage was disabled");
      taskSendMessage.disable();
      taskSendMessageEnabled = false;

      // and allow measurements after
      taskGetMeasurementsAlertEnabled = true;
    }
  }

  // acknowledge sleep command
  if (stringReceivePayload == "sleep")
  {
    sleepCommandReceived = true;
    logln("Sleep command was received!");
  }

  // check node status
  if (onlineMode)
  {
    logln("onlineMode");
  }
  else if (offlineMode)
  {
    logln("offlineMode");
  }
  if (alertMode)
  {
    logln("alertMode");
  }
}

// functions for connection changes
void newConnectionCallback(uint32_t nodeId)
{
  logln("--> startHere: New Connection, nodeId = " + String(nodeId));
}

void changedConnectionCallback()
{
  logln("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset)
{
  logln("Adjusted time " + String(mesh.getNodeTime()) + ". Offset = " + String(offset));
}

void startMesh()
{
  logln("Starting mesh...");
  WiFi.begin();
  delay(1);
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  mesh.setContainsRoot();

  // add scheduled tasks
  userScheduler.addTask(taskWarmUp);
  userScheduler.addTask(taskGetMeasurements);
  userScheduler.addTask(taskGetMeasurementsAlert);
  userScheduler.addTask(taskSendMessage);
}

void stopMesh()
{
  logln("Stopping mesh...");
  mesh.stop();
  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true, false);
  delay(1);
}

/*=================================================================================================================
  Setup
==================================================================================================================*/
void setup()
{
  // Manual Testing ===============================================================================================
  if (switchAlert)
  {
    offAlert = false;
    onAlert = true;
  }

  // Serial Setup  ==========================================================================================================================================
  Serial.begin(115200);
  Serial.println("setup() was called");
  delay(1000);

  // SD Card Setup ==========================================================================================================================================
  if (!sd.begin(chipSelect, SPI_HALF_SPEED))
  {
    Serial.println("SD Card failed to initialize");
    // sd.initErrorHalt();
  }
  else
  {
    logln("SD Card Module connected successfully!");
    sdWorks = true;
    numDataPackets = SDCardRead("numDataPackets.txt").toInt();
  }

  // if viewLogMode, display contents of log file and pause program
  if (viewLogMode)
  {
    Serial.println(SDCardRead("log.txt"));
    while (true)
      ;
  }

  // initialize SD card if initSD
  if (initSD)
  {
    // temporary cleaning of SD Card for testing
    logln("Initializing SD Card and removing files...");

    // remove all old payloads
    if (sd.exists("numDataPackets.txt") && sd.exists("payloadToBeTransmitted.txt"))
    {
      int lastPayloadNumber = SDCardRead("numDataPackets.txt").toInt();
      for (int i = SDCardRead("payloadToBeTransmitted.txt").toInt(); i <= lastPayloadNumber; i++)
      {
        Serial.println(String(i) + "/" + String(lastPayloadNumber));
        String StringFileName = String(i) + ".txt";        // get payload file name without ".txt"                                // add ".txt"
        const char *charFileName = StringFileName.c_str(); // convert string filename number to const char
        sd.remove(charFileName);
      }
    }
    // remove data file
    sd.remove("data.txt");

    // remove old log file
    sd.remove("log.txt");

    // remove old packet building files
    sd.remove("numDataPackets.txt");
    sd.remove("payloadFileName.txt");
    sd.remove("payloadToBeTransmitted.txt");

    // initialize packet building files and first payload
    SDCardWrite("numDataPackets.txt", "0");
    SDCardWrite("payloadFileName.txt", "1");
    SDCardWrite("1.txt", "");
    SDCardWrite("payloadToBeTransmitted.txt", "1");

    // check initialized files
    logln("numDataPackets.txt: " + SDCardRead("numDataPackets.txt"));
    logln("payloadFileName.txt: " + SDCardRead("payloadFileName.txt"));
    logln("payloadToBeTransmitted.txt: " + SDCardRead("payloadToBeTransmitted.txt"));

    // change initSD to false and restart MCU
    logln("Initialized SD Card and removed old files!");
    logln("Change initSD to false and re-upload code to node!");
    while (true)
      ;
  }

  // Timestamp Setup ==========================================================================================================================================
  rtc.setTime(second, minute, hour, day, month, year);
  logln("Default time is " + rtc.getDateTime());

  // Sensor Setup ==========================================================================================================================================
  // INA219
  while (!ina219.begin())
  {
    logln("INA219 NOT detected!");
    delay(1000);
  }
  logln("INA219 connected successfully!");

  // DFRobot BME680
  gas_sensor.dev_id = BME680_I2C_ADDR_PRIMARY;
  gas_sensor.intf = BME680_I2C_INTF;
  gas_sensor.read = user_i2c_read;
  gas_sensor.write = user_i2c_write;
  gas_sensor.delay_ms = user_delay_ms;
  /* amb_temp can be set to 25 prior to configuring the gas sensor
   * or by performing a few temperature readings without operating the gas sensor.
   */
  gas_sensor.amb_temp = 25;

  Wire.begin();
  uint8_t rslt = 1;
  while (rslt != 0)
  {
    rslt = bme680_init(&gas_sensor);
    init_bme();
    if (rslt != 0)
    {
      logln("DFRobot BME680 NOT detected!");
      delay(1000);
    }
  }
  logln("DFRobot BME680 connected successfully!");

  // DFRobot MiCS-4514
  while (!mics.begin())
  {
    logln("DFRobot MiCS-4514 NOT detected!");
    delay(1000);
  }
  logln("DFRobot MiCS-4514 connected successfully!");

  uint8_t mode = mics.getPowerState();
  if (mode == SLEEP_MODE)
  {
    mics.wakeUpMode();
    logln("DFRobot MiCS-4514 woke up successfully!");
  }
  else
  {
    logln("DFRobot MiCS-4514 is already awake!");
  }

  // MiCS-VZ89TE
  bool micsVzBegin = false;
  while (!micsVzBegin)
  {
    micsVZ.getVersion();
    if (int(micsVZ.getYear()) != 255)
    {
      micsVzBegin = true;
    }
    else
    {
      logln("MiCS-VZ89TE NOT detected!");
      delay(1000);
    }
  }
  logln("MiCS-VZ89TE connected successfully!");

  // ULPSMSO2 986-006
  logln("ULPSMSO2 986-006 VGAS = " + String(analogRead(VGAS) * 3.3 / 4096) + "V");
  logln("ULPSMSO2 986-006 VREF = " + String(analogRead(VREF) * 3.3 / 4096) + "V");

  // Sensirion SPS30
  // set driver debug level
  sps30.EnableDebugging(DEBUG_);

  // Begin communication channel
  SP30_COMMS.begin();

  if (sps30.begin(&SP30_COMMS) == false)
  {
    Errorloop((char *)"SPS30 could not set I2C communication channel.", 0);
  }

  // check for SPS30 connection
  if (!sps30.probe())
    Errorloop((char *)"SPS30 NOT detected!", 0);
  else
    logln(F("SPS30 connected successfully!"));

  // reset SPS30 connection
  if (!sps30.reset())
    Errorloop((char *)"SPS30 could not reset.", 0);

  // read device info
  // GetDeviceInfo();

  // start measurement
  if (sps30.start())
    logln(F("SPS30 woke up successfully!"));
  else
    Errorloop((char *)"SPS30 could not start measurements.", 0);

  // serialTrigger((char *)"Hit <enter> to continue reading.");

  if (sps30.I2C_expect() == 4)
    logln(F(" !!! Due to I2C buffersize only the SPS30 MASS concentration is available!\n"));

  // Mesh Setup ==========================================================================================================================================
  if (!meshActive)
  {
    startMesh();
    meshActive = true;
  }

  // set up first payload on reset
  nodeID = mesh.getNodeId();
  logln("nodeID = " + String(nodeID));
  if (SDCardRead("payloadFileName.txt") == "1" && SDCardRead("1.txt") == "")
  {
    SDCardWrite("1.txt", (String(nodeID) + "/").c_str());
  }

  // if it doesn't exist and it's not the first payload
  // make that file and append node ID
  if (!sd.exists(SDCardRead("payloadFileName.txt") + ".txt") && SDCardRead("payloadFileName.txt") != "1")
  {
    SDCardWrite((SDCardRead("payloadFileName.txt") + ".txt").c_str(), (String(nodeID) + "/").c_str());
  }
  logln("Finished setup!");
}

/*=================================================================================================================
  Loop
==================================================================================================================*/
void loop()
{
  // update mesh when active
  if (meshActive)
  {
    mesh.update();
  }

  // main modes
  if (warmUpMode)
  {
    if (taskWarmUpEnabled)
    {
      taskWarmUpEnabled = false;
      logln("taskWarmUp was enabled");
      taskWarmUp.enable();
    }
  }

  if (offlineMode)
  {
    if (meshActive)
    {
      stopMesh();
      meshActive = false;

      logln("taskWarmUp was disabled");
      taskWarmUp.disable();
      taskGetMeasurements.disable();
      taskGetMeasurementsAlert.disable();
      taskSendMessage.disable();
    }
    getMeasurements();
    if (offlineMode)
    {
      delay(60000);
    }
    else if (onlineMode)
    {
      delay(30000);
    }
  }

  if (onlineMode)
  {
    if (!meshActive)
    {
      startMesh();
      meshActive = true;
    }

    // check if node is actually connected to mesh every 1 minutes
    if (millis() - timeChecked >= 60000 * 1)
    {
      timeChecked = millis();
      currentNodes.clear();
      for (auto v : mesh.getNodeList())
      {
        // store nodes in currentNodes
        currentNodes.push_back(String(v));
        currentNodes.sort();
        currentNodes.unique();
      }
      // report current mesh status
      logln("Current nodes in mesh = ");
      for (auto v : currentNodes)
      {
        log(v);
        log(", ");
      }
      logln();

      // if node list is empty, try to restart mesh
      if (currentNodes.empty())
      {
        // if node list is still empty after 30 minutes
        // hard reset node
        if (millis() - lastConnect >= 1800000)
        {
          String entry2 = rtc.getDateTime() + "    " + "Abort() was called due to prolonged disconnection to mesh!";
          SDCardAppend("log.txt", (entry2).c_str());
          delay(1000);
          abort();
        }

        taskGetMeasurementsAlert.disable();
        taskGetMeasurementsAlertOngoing = false;
        taskGetMeasurements.disable();
        taskSendMessage.disable();
        stopMesh();
        delay(10);
        startMesh();
        offlineMode = false;
        onlineMode = true;
        taskGetMeasurementsAlertEnabled = true;
      }
      else
      {
        taskSendMessageEnabled = true;
      }
    }
  }

  // enable tasks once
  if (taskGetMeasurementsEnabled && ((millis() - modeTime) >= 60000))
  {
    modeTime = millis();
    taskGetMeasurementsEnabled = false;
    taskGetMeasurements.enable();
    logln("taskGetMeasurements was enabled");
  }

  if (taskSendMessageEnabled)
  {
    taskSendMessageEnabled = false;
    taskSendMessage.enable();
    logln("taskSendMessage was enabled");
  }

  if (taskGetMeasurementsAlertEnabled && !taskGetMeasurementsAlertOngoing)
  {
    // disable original taskGetMeasurements
    taskGetMeasurements.disable();
    logln("taskGetMeasurements was disabled");

    // enable tasks for alertMode
    taskGetMeasurementsAlertEnabled = false;
    taskGetMeasurementsAlert.enable();
    taskGetMeasurementsAlertOngoing = true;
    logln("taskGetMeasurementsAlert was enabled");
  }
}
