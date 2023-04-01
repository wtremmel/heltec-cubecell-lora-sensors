// #define DEBUG



#include "Arduino.h"
#include <Wire.h>
#include <ArduinoLog.h>

#if defined(ARDUINO_ARCH_ASR650X)
#include "innerWdt.h"
#include "LoRaWan_APP.h"
#include "cubecell.h"
#elif defined(ARDUINO_ARCH_ESP32)
// #include <ESP32_LoRaWAN.h>
// #include "esp32-hal-adc.h"
#include "heltec.h"
#endif


#include <CayenneLPP.h>
CayenneLPP lpp(51);

// Sensor Libraries
#include "Adafruit_Si7021.h"
#include "Adafruit_BME280.h"
#include "Adafruit_TSL2561_U.h"
#include <Adafruit_ADS1X15.h>

#include <Max44009.h>

#define SOLAR_ONLY 1

#if defined(SOLAR_ONLY)
#define BATTERY_RECHARGABLE 1
#define HAS_RGB 0
#define SHUTDOWN_VOLTAGE 0 // no shutdown
#define RESTART_VOLTAGE 0  // 3.0V
#define HIBERNATION_SLEEPTIME 60*1000*2  // 2 minutes
#define CYCLE_MIN  60*1000  // 1 minute
#define CYCLE_MAX 60*1000*20  // 10 minutes
#define VOLTAGE_MAX 4100  // 4.1V
#define VOLTAGE_MIN 3300  // 3.0V

#elif defined(CubeCell_HalfAA)
#define BATTERY_RECHARGABLE 0
#define HAS_RGB 0
#define SHUTDOWN_VOLTAGE 0 // no shutdown

#define RESTART_VOLTAGE 0  // 3.0V
#define HIBERNATION_SLEEPTIME 60*1000*5  // 5 minutes
#define CYCLE_MIN  60000  // 1 minute
#define CYCLE_MAX 240000  // 4 minutes
#define VOLTAGE_MAX 3330  // 3.9V
#define VOLTAGE_MIN 3270  // 3.0V

#else
#define BATTERY_RECHARGABLE 1
#define HAS_RGB 1

// #define SHUTDOWN_VOLTAGE 3500 // 3.5V
#define SHUTDOWN_VOLTAGE 2800 // 3.5V
// #define RESTART_VOLTAGE 3600  // 3.6V
#define RESTART_VOLTAGE 3000  // 3.6V
#define HIBERNATION_SLEEPTIME 60*1000*120  // 120 minutes
#define CYCLE_MIN  2*60*1000  // 2 minutes
#define CYCLE_MAX 60*1000*20  // 20 minutes
#define VOLTAGE_MAX 4200  // 4.2V
#define VOLTAGE_MIN RESTART_VOLTAGE  // 3.0V
#endif

#if defined(DEBUG)
#define LOGLEVEL LOG_LEVEL_VERBOSE
#else
#define LOGLEVEL LOG_LEVEL_SILENT
#endif

#if HAS_RGB
#include "CubeCell_NeoPixel.h"
#endif

uint32_t cycle_min = CYCLE_MIN,
  cycle_max = CYCLE_MAX,
  voltage_min = VOLTAGE_MIN,
  voltage_max = VOLTAGE_MAX,
  restart_voltage = RESTART_VOLTAGE,
  shutdown_voltage = SHUTDOWN_VOLTAGE,
  hibernation_sleeptime = HIBERNATION_SLEEPTIME;


uint16_t lastV = 0;
bool hibernationMode = false;
uint32_t last_cycle = HIBERNATION_SLEEPTIME;


// Global Objects
Adafruit_Si7021 si7021;
Max44009 gy49(0x4a);
Adafruit_BME280 bme280;
Adafruit_TSL2561_Unified tsl2561 = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT);
#if HAS_RGB
CubeCell_NeoPixel pixels(1, RGB, NEO_GRB + NEO_KHZ800);
#endif
Adafruit_ADS1115 ads1115;


bool bme280_found = false;
bool voltage_found = true;
bool gy49_found = false;
bool ads1115_found = false;
bool ads1115_initialized = false;
bool tsl2561_initialized = false;
bool tsl2561_found = false;

bool setup_complete = false;
bool pixels_initalized = false;
bool drain_battery = false;

uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t devAddr;
/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS; 

/*the application data transmission duty cycle.  value in [ms].*/
#if BATTERY_RECHARGABLE
uint32_t appTxDutyCycle = CYCLE_MIN;
bool variableDutyCycle = true;
#else
uint32_t appTxDutyCycle = CYCLE_MAX;
bool variableDutyCycle = false;
#endif
/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;

uint8_t ledr = 0, ledg = 0, ledb = 0;
bool ledon = false;

/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

// external power functions
void vext_power(bool on) {
  if (on && !hibernationMode) {
    digitalWrite(Vext,LOW);
  } else {
    digitalWrite(Vext,HIGH);
  }
}

#if defined(ARDUINO_ARCH_ESP32)
uint16_t getBatteryVoltage() {
  analogSetCycles(8);
  analogSetSamples(1);
  analogSetClockDiv(1);
  analogSetPinAttenuation(37,ADC_11db);
  adcAttachPin(37);
  adcStart(37);
  // while(adcBusy(37));
  uint16_t v = (analogRead(37)*0.0025*1000);
  // adcEnd(37);

  return v;
}
#endif


void set_led(uint8_t r, uint8_t g, uint8_t b) {
  // switch on power
#if HAS_RGB
  vext_power(true);

  Log.verbose(F("set_led(%d,%d,%d)"),r,g,b);
  if (!pixels_initalized){
    pixels.begin();
    pixels_initalized = true;
  }

  if (r == 0 && g == 0 && b == 0) {
    pixels.clear();
    pixels.show();
  } else {
    pixels.setPixelColor(0, pixels.Color(r,g,b));
    pixels.show();
    // delay(10*1000);
  }
#endif
}


void set_hibernation(bool on) {
  if (on) {
    if (!hibernationMode) {
      hibernationMode = true;
      Log.notice(F("Hibernation mode now on"));
      set_led(0,0,0);
      vext_power(false);
      appTxDutyCycle = hibernation_sleeptime;
      drain_battery = false;
      variableDutyCycle = false;
    } else {
      Log.verbose(F("Hibernation mode already on, doing nothing"));
    }
  } else {
    if (hibernationMode) {
      hibernationMode = false;
      appTxDutyCycle = cycle_min;
      variableDutyCycle = true;
      vext_power(true);
      Log.notice(F("Hibernation mode now off"));
    } else {
      Log.verbose(F("Hibernation mode already off, doing nothing"));
    }
  }
  Log.verbose(F("set_hibernation: Duty cycle: %d s"),int(appTxDutyCycle / 1000));

}

//
// Scan for sensors
//
void setup_i2c() {
  byte error, address;
  unsigned int devices=0;

// 0x29 TSL45315 (Light)
// 0x38 VEML6070 (Light)
// 0x39 TSL2561
// 0x40 SI7021
// 0x48 4*AD converter
// 0x4a GY49 or MAX44009 Light Sensor
// 0x50 PCF8583P
// 0x57 ATMEL732
// 0x68 DS3231 Clock
// 0x76 BME280
// 0x77 BME680 (also BMP180)

  Log.verbose("Scanning i2c bus");
  Wire.begin();
  for(address = 1; address < 127; address++ ) {
    Log.verbose(F("Trying 0x%x"),address);
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Log.verbose(F("I2C device found at address 0x%x !"),address);
      devices++;

      if (address==0x39) {
        // TSL2561
        Log.notice(F("TSL2561 found at 0x%x"),address);
        if (tsl2561.begin()) {
          tsl2561.enableAutoRange(true);
          tsl2561.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
        }
        tsl2561_found = true;
      }
      
      if (address == 0x48) {
        // AD-converter
        Log.notice(F("ADS1115 found at 0x%x"),address);
        if (!ads1115_initialized) {
          // initialize only once as it comsumes memory
          Log.notice(F("ADS1115 initialized"));
          ads1115.begin();
          ads1115_initialized = true;
        }
        ads1115_found = true;
      }

      if (address == 0x4a) {
        // GY-49
        float l = gy49.getLux();
        if (! gy49.getError()) {
          gy49_found = true;
          Log.notice(F("GY49 found at 0x%x, current Lux %F"),address,l);
        }
      }

      if (address == 0x76) {
        // BME280
        bme280_found = bme280.begin(address);
        Log.verbose(F("BME280 found? %T"),bme280_found);
      }
    }
  }
  Log.verbose(F("i2c bus scanning complete, %d devices"),devices);
}


// BME280 routines
void read_bme280() {
  Log.verbose(F("read_bme280"));
  lpp.addTemperature(1,bme280.readTemperature());
  lpp.addRelativeHumidity(2,bme280.readHumidity());
  lpp.addBarometricPressure(3,bme280.readPressure() / 100.0F);
}

void read_gy49() {
  float l;
  Log.verbose(F("read_gy49"));
  l = gy49.getLux();
  if (l < 65000)
    lpp.addLuminosity(4,l);
}

void read_ads1115() {
  for (int i=0;i<4;i++) {
    uint16_t r = ads1115.readADC_SingleEnded(i);
    lpp.addLuminosity(20+i, r);
  }
}

void read_tsl2561() {
  sensors_event_t event;
  Log.verbose(F("read_tsl2561"));
  tsl2561.getEvent(&event);
  lpp.addLuminosity(4,(float)event.light);
}

void print_timer_values() {
  Log.verbose(F("cycle_min = %d"),cycle_min);
  Log.verbose(F("cycle_max = %d"),cycle_max);
  Log.verbose(F("hibernation_sleeptime = %d"),hibernation_sleeptime);
  Log.verbose(F("voltage_min = %d"),voltage_min);
  Log.verbose(F("voltage_max = %d"),voltage_max);
  Log.verbose(F("shutdown_voltage = %d"),shutdown_voltage);
  Log.verbose(F("restart_voltage = %d"),restart_voltage);
  Log.verbose(F("Duty cycle: %d s"),int(appTxDutyCycle / 1000));

}

// Battery voltage
void read_voltage() {
  uint16_t v = getBatteryVoltage();
  lpp.addAnalogInput(5,(float)v / 1000.0);
  Log.verbose(F("Voltage: %d"),v);
  if (!hibernationMode && v <= shutdown_voltage) {
    Log.notice(F("Voltage %d <= Shutdown voltage (%d), hibernation mode"),v,shutdown_voltage);
    set_hibernation(true);
    lastV = v;
  }
  if (hibernationMode && v >= restart_voltage) {
    set_hibernation(false);
  }

  if (hibernationMode) {
    if (v < lastV)
      appTxDutyCycle += hibernation_sleeptime;
    if (v > lastV)
      appTxDutyCycle = hibernation_sleeptime;

    lastV = v;
    Log.verbose(F("read_voltage: Duty cycle: %d s"),int(appTxDutyCycle / 1000));

  }
  else if (variableDutyCycle) {
    // duty cycle depending on voltage
    // max duty cycle = 4 minutes = 240000
    // min duty cycle = 1 minute = 60000
    // min voltage = 3000
    // max voltage = 3900


    // ((t2-t1)/(v2-v1))*(v-v1)+t1
    long int cycle = (((long)cycle_min - (long)cycle_max)/
      ((long)voltage_max-(long)voltage_min)) *
      (v - (long)voltage_min) + (long)cycle_max;

    print_timer_values();
    Log.verbose(F("cycle = %d"),cycle);

    if (cycle < (long int)cycle_min)
      appTxDutyCycle = cycle_min;
    else if (cycle > (long int)cycle_max)
      appTxDutyCycle = cycle_max;
    else
      appTxDutyCycle = abs(cycle);

    Log.verbose(F("read_voltage2: Duty cycle: %d s"),int(appTxDutyCycle / 1000));
  }
}

// Sensor routines
void read_sensors() {
  lpp.reset();

  // switch on power
  vext_power(true);
  set_led(ledr,ledg,ledb);

  delay(100);

  if (voltage_found) {
    read_voltage();
  }

  // initialize sensors

  if (!hibernationMode) {
    setup_i2c();

    if (bme280_found) {
      read_bme280();
    }
    if (gy49_found) {
      read_gy49();
    }
    if (ads1115_found) {
      read_ads1115();
    }
    if (tsl2561_found) {
      read_tsl2561();
    }
    Wire.end();
  }
}



void setup_serial() {
#if defined(DEBUG)
  Serial.begin(115200);
#endif
}


#if defined(DEBUG)
// Logging helper routines
void printTimestamp(Print* _logOutput, int logLevel) {
  static char c[12];
  // sprintf(c, "%l ", TimerGetCurrentTime());
  sprintf(c, "%d ", millis());
  _logOutput->print(c);
}

void printNewline(Print* _logOutput, int logLevel) {
  _logOutput->print('\n');
}

void setup_logging() {
  Log.begin(LOGLEVEL, &Serial);
  Log.setPrefix(printTimestamp);
  Log.setSuffix(printNewline);
  Log.verbose("Logging has started");
}
#else
void setup_logging() {}
#endif

void setup_lora() {
  Log.verbose(F("setup_lora: start"));
  deviceState = DEVICE_STATE_INIT;
	LoRaWAN.ifskipjoin();
}


void setup_check_voltage() {
  // Check if voltage is above restart_voltage
  uint16_t v = getBatteryVoltage();
  Log.verbose(F("Voltage: %d"),v);
  if (v <= shutdown_voltage) {
    set_hibernation(true);
  }
}

void setup_chipid() {
  uint64_t chipID = 0;
#if defined(ARDUINO_ARCH_ASR650X)
  chipID=getID();
  Log.notice(F("Chip ID = %X%x"),
    (uint32_t)(chipID>>32),(uint32_t)chipID);
#elif defined(ARDUINO_ARCH_ESP32)
  chipID=ESP.getEfuseMac();
  Log.notice(F("ESP32 Chip model = %s Rev %d\n"), ESP.getChipModel(), ESP.getChipRevision());
  Log.notice(F("This chip has %d cores\n"), ESP.getChipCores());
  Log.notice(F("ESP32ChipID=%04X"),(uint16_t)(chipID>>32));//print High 2bytes
  Log.notice(F("%08X\r\n"),(uint32_t)chipID);//print Low 4bytes.
#endif
}

void setup_battery() {
#if defined(ARDUINO_ARCH_ASR650X)
#elif defined(ARDUINO_ARCH_ESP32)
#endif
}

void setup() {
#if defined(ARDUINO_ARCH_ASR650X)
  // Turn on watchdog
  innerWdtEnable(true);
#endif

  ledr = 0;
  ledg = 0;
  ledb = 0;

  setup_serial();
  delay(5000);
  setup_logging();
  Log.verbose(F("setup(): Logging started"));
  setup_chipid();
  setup_check_voltage();

  // Turn on power for devices
  pinMode(Vext,OUTPUT);
  vext_power(true);
  set_led(ledr,ledg,ledb);

  setup_i2c();
  setup_lora();
  print_timer_values();
}

static void prepareTxFrame( ) {
  read_sensors();

  appDataSize = lpp.getSize();
  memcpy(appData,lpp.getBuffer(),appDataSize);
}

// -------------- Command Processing -----------------
void process_system_led_command(unsigned char len, unsigned char *buffer) {
  if (len == 0) {
    Log.error(F("Zero length LED command"));
    return;
  } else {
    Log.verbose(F("Processing LED command"));
  }

  switch (buffer[0]) {
    case 0x00:
      ledr = 0;
      ledg = 0;
      ledb = 0;
      set_led(0,0,0);
      break;
    case 0x01:
      ledr = 255;
      ledg = 255;
      ledb = 255;
      set_led(255,255,255);
      break;
#if HAS_RGB
    case 0x02:
      if (len == 4) {
        // do rgb magic
        ledr = buffer[1];
        ledg = buffer[2];
        ledb = buffer[3];
        set_led(ledr,ledg,ledb);
      } else {
        Log.error(F("Missing RGB values for LED. Len = %d"),len);
      }
      break;
#endif
    default:
      Log.error(F("Unknown LED command %d"), buffer[0]);
      break;
  }
}

void process_system_power_command(unsigned char len, unsigned char *buffer) {
  if (len == 0) {
    Log.error(F("Zero length power command"));
  } else {
    Log.verbose(F("Processing power command"));
  }

  switch (buffer[0]) {
    case 0x01:
      drain_battery = false;
      Log.verbose(F("Power save to default"));
      break;
    case 0x10:
      set_hibernation(true);
      break;
    case 0x11:
      set_hibernation(false);
      break;
#if BATTERY_RECHARGABLE
    case 0xff:
      drain_battery = true;
      Log.verbose(F("Drain battery on"));
      break;
#endif
    default:
      Log.error(F("Unknown power command %d"),buffer[0]);
      break;
  }
}

void set_default_timers() {
  cycle_min = CYCLE_MIN;
  cycle_max = CYCLE_MAX;
  voltage_min = VOLTAGE_MIN;
  voltage_max = VOLTAGE_MAX;
  restart_voltage = RESTART_VOLTAGE;
  shutdown_voltage = SHUTDOWN_VOLTAGE;
  hibernation_sleeptime = HIBERNATION_SLEEPTIME;
}



void process_system_delay_command(unsigned char len, unsigned char *buffer) {
  if (len != 1) {
    Log.error(F("Len of delay command != 1"));
  } else {
    Log.verbose(F("Processing delay command"));
  }

  if (buffer[0] == 0) {
#if !BATTERY_RECHARGABLE
    variableDutyCycle = true;
    Log.verbose(F("Duty cycle variable"));
#endif
  } else {
    variableDutyCycle = false;
    appTxDutyCycle = buffer[0] * 1000 * 5;
    Log.verbose(F("Duty cycle %d seconds"),appTxDutyCycle / 1000);
  }
}

void process_system_timer_command(unsigned char len, unsigned char *buffer) {
  if (len <= 1) {
    Log.error(F("Len of timer command <= 1"));
  } else {
    Log.verbose(F("Processing timer command"));
  }

  switch(buffer[0]){
    case 0x00:
      set_default_timers();
      break;
    case 0x01:
      cycle_min = 60*1000*buffer[1];
      break;
    case 0x02:
      cycle_max = 60*1000*buffer[1];
      break;
    case 0x03:
      hibernation_sleeptime = 60*1000*buffer[1];
      break;
    case 0x11:
      voltage_min = 100 * buffer[1];
      break;
    case 0x12:
      voltage_max = 100 * buffer[1];
      break;
    case 0x13:
      shutdown_voltage = 100 * buffer[1];
      break;
    case 0x14:
      restart_voltage = 100 * buffer[1];
      break;
    case 0xff:
      if (len == 7) {
        cycle_min = 60*1000*buffer[1];
        cycle_max = 60*1000*buffer[2];
        hibernation_sleeptime = 60*1000*buffer[3];
        voltage_min = 100 * buffer[4];
        voltage_max = 100 * buffer[5];
        shutdown_voltage = 100 * buffer[6];
        restart_voltage = 100 * buffer[7];
      } else {
        Log.error(F("Only %d arguments, needed 7"),len);
      }
      break;
    default:
      Log.error(F("Unknown timer command %X"),buffer[0]);
  }
  // sanity Check
  if (cycle_min > cycle_max ||
    voltage_min >= voltage_max ||
    shutdown_voltage > restart_voltage) {
      Log.error(F("Sanity check failed, loading defaults"));
      set_default_timers();
    }
  print_timer_values();
}

void process_system_command(unsigned char len, unsigned char *buffer) {
  if (len == 0) {
    Log.error(F("Zero length system command"));
    return;
  } else {
    Log.verbose(F("Processing system command"));
  }
  switch (buffer[0]) {
    case 0x01:
      process_system_power_command(len-1,buffer+1);
      break;
    case 0x02:
      process_system_delay_command(len-1,buffer+1);
      break;
    case 0x03:
      process_system_led_command(len-1,buffer+1);
      break;
    case 0x08:
      process_system_timer_command(len-1,buffer+1);
      break;
    case 0xff:
      // Reboots
      Log.notice(F("Executing reboot command"));
      delay(100);
#if defined(ARDUINO_ARCH_ASR650X)
      HW_Reset(0);
#elif defined(ARDUINO_ARCH_ESP32)
      ESP.restart();
#endif
    default:
      Log.error(F("Unknown system command %d"),buffer[0]);
      break;
  }
}

void process_sensor_bme280(unsigned char len, unsigned char *buffer) {
  if (len == 0) {
    Log.error(F("Zero length bme280 command"));
    return;
  }

}

void process_sensor_command(unsigned char len, unsigned char *buffer) {
  if (len == 0) {
    Log.error(F("Zero length sensor command"));
    return;
  }
  switch (buffer[0]) {
    case 0x11:
      process_sensor_bme280(len-1,buffer+1);
      break;
    default:
      Log.error(F("Unknown sensor command %d"),buffer[0]);
      break;
  }
}

void process_received_lora(unsigned char len, unsigned char *buffer) {
  if (len == 0)
    return;

  Log.verbose(F("Processing %d bytes of received data"),len);
  switch (buffer[0]) {
    case 0:
      process_system_command(len-1,buffer+1);
      break;
    case 1:
      process_sensor_command(len-1,buffer+1);
      break;
    default:
      Log.error(F("Unknown command %d"),buffer[0]);
      break;
  }
}


void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  Log.verbose(F("+REV DATA:%s,RXSIZE %d,PORT %d\r\n"),
    mcpsIndication->RxSlot?"RXWIN2":"RXWIN1",
    mcpsIndication->BufferSize,
    mcpsIndication->Port);
  process_received_lora(mcpsIndication->BufferSize, mcpsIndication->Buffer);
}



void loop() {
  setup_complete = true;
  switch( deviceState )
	{
		case DEVICE_STATE_INIT:
		{
			LoRaWAN.generateDeveuiByChipID();
			printDevParam();
			LoRaWAN.init(loraWanClass,loraWanRegion);
			deviceState = DEVICE_STATE_JOIN;
			break;
		}
		case DEVICE_STATE_JOIN:
		{
			LoRaWAN.join();
			break;
		}
		case DEVICE_STATE_SEND:
		{
			// prepareTxFrame( appPort );
      prepareTxFrame();
			LoRaWAN.send();
			deviceState = DEVICE_STATE_CYCLE;
			break;
		}
		case DEVICE_STATE_CYCLE:
		{
			// Schedule next packet transmission
			txDutyCycleTime = appTxDutyCycle;
      Log.verbose(F("DEVICE_STATE_CYCLE: Duty cycle: %d s"),int(appTxDutyCycle / 1000));


			LoRaWAN.cycle(txDutyCycleTime);
			deviceState = DEVICE_STATE_SLEEP;
			break;
		}
		case DEVICE_STATE_SLEEP:
		{
      // switch off power
      if (!drain_battery)
        vext_power(false);
      // Log.verbose(F("Sleeping - txDutyCycleTime = %d"),txDutyCycleTime);
      // delay(10);
			LoRaWAN.sleep();
			break;
		}
		default:
		{
			deviceState = DEVICE_STATE_INIT;
			break;
		}
	}

}
