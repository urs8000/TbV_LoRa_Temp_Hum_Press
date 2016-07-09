/*******************************************************************************
  This program is made of several snippets and configured for the TvB board from belgum
  see: https://eth0maz.wordpress.com/2016/05/27/lorawan-iot-node/
  but unfortunately this version of the board lacks of the tree connecions: DIO1, DIO2, Reset
  which have to be made by hand.  see TvB_final.jpg on git
  
  
  Sketch uses 38,646 bytes (29%) of program storage space. Maximum is 130,048 bytes.
  
  for the next version the HTU and BMP will be replaced with one BME280

  
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <LowPower.h>
#include "HTU21D.h"
#include "SFE_BMP180.h"



// LoRaWAN NwkSKey, network session key
// Get these keys from an activated device using: 
// ttnctl devices info <DEVICE> 
// 
// application "your reminder"
static const u4_t DEVADDR             = 0xXXXXXXXX; 
static const u1_t PROGMEM NWKSKEY[16] = { you own key };
static const u1_t PROGMEM APPSKEY[16] = { you own key };

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds        sleep 37 x 8 Seconds, then go ahead ... do this in sleep routine
const unsigned TX_INTERVAL = 1;            // so this is obvious

// Pin mapping
// Pin mapping (modified)
const lmic_pinmap lmic_pins = {
   .nss  = 4,
   .rxtx = LMIC_UNUSED_PIN,
   .rst  = 11,                          // 11, 
   .dio  = {14, 12, 13},                // 12 & 13 bases on my own soldering definition
};

// restrict to channel0 and SF7 if uncommented; otherwise all channels & SF12
// #define CHANNEL0

// enable debug statements to Serial Monitor if uncommented
// #define DEBUG1
 
byte buffer[32];
int counter      = 1;
int VoltagePin   = A4;               // read the battery voltage
int NumberOf8Sec = 37;               // sleep 37*8Sec = 296Sec = 4.933Min 

//declare output pins for powering the sensors - be aware of the sensors timing and current
int power = 30;                                

boolean HTUinit = true;              // sad, not jet implemented in the library
boolean BMPinit = false;

float HTU_h = 0;
float HTU_t = 0;

char status;
double BMP_T;
double BMP_P;

//Create an instance of the objects for both sensors
HTU21D myHumidity;
SFE_BMP180 pressure;

// Pin 20 & 21 are red and blue led  (next version should have a RGB led)
   int red  = 20;
   int blue = 21;
   
   
//----------------------------------------------------------------------------------------
void onEvent (ev_t ev) {
  #ifdef DEBUG1
    Serial.print("os_time: ");
    Serial.println(os_getTime());
    Serial.print("Time: ");
    Serial.println(millis() / 1000);
    Serial.print("event: ");
    Serial.println(ev);
   #endif 
    // blink_x(ev);
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            blink_x(red, 1);
            #ifdef DEBUG1
              Serial.println(F("EV_SCAN_TIMEOUT"));
            #endif  
            break;
        case EV_BEACON_FOUND:
            blink_x(blue, 2);
            #ifdef DEBUG1
              Serial.println(F("EV_BEACON_FOUND"));
            #endif  
            break;
        case EV_BEACON_MISSED:
            blink_x(red, 2);
            #ifdef DEBUG1
              Serial.println(F("EV_BEACON_MISSED"));
            #endif  
            break;
        case EV_BEACON_TRACKED:
            blink_x(blue, 2);
            #ifdef DEBUG1
              Serial.println(F("EV_BEACON_TRACKED"));
            #endif  
            break;
        case EV_JOINING:
            blink_x(blue, 3);
            #ifdef DEBUG1
              Serial.println(F("EV_JOINING"));
            #endif  
            break;
        case EV_JOINED:
            blink_x(blue, 3);
            #ifdef DEBUG1
              Serial.println(F("EV_JOINED"));
            #endif  
            break;
        case EV_RFU1:
            blink_x(red, 2);
            #ifdef DEBUG1
              Serial.println(F("EV_RFU1"));
            #endif  
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            blink_x(red, 5);
            #ifdef DEBUG1
              Serial.println(F("EV_REJOIN_FAILED"));
            #endif  
            break;
        case EV_TXCOMPLETE:   // do NOT sleep, befor you had an answer and yes I accept, that it does not sleep in any other cases
            #ifdef DEBUG1
              Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            #endif  
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                #ifdef DEBUG1
                  Serial.print(F("Data Received: "));
                  Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                  Serial.println();
                 #endif
                blink_x(blue, LMIC.dataLen);
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            delay(200);
            do_sleep();            
            break;
        case EV_LOST_TSYNC:
            #ifdef DEBUG1
              Serial.println(F("EV_LOST_TSYNC"));
            #endif  
            break;
        case EV_RESET:
            #ifdef DEBUG1
              Serial.println(F("EV_RESET"));
            #endif  
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            blink_x(blue, 1);
            #ifdef DEBUG1
              Serial.println(F("EV_RXCOMPLETE"));
            #endif  
            break;
        case EV_LINK_DEAD:
            blink_x(red, 1);
            #ifdef DEBUG1
              Serial.println(F("EV_LINK_DEAD"));
            #endif  
            break;
        case EV_LINK_ALIVE:
            #ifdef DEBUG1
              Serial.println(F("EV_LINK_ALIVE"));
            #endif  
            break;
         default:
            #ifdef DEBUG1
              Serial.println(F("Unknown event"));
            #endif
            break;
    }
    #ifdef DEBUG1
      Serial.println();
    #endif  
}


void do_send(osjob_t* j) {
  readSensors();
  float Battery = Batt_Voltage(880, 102);
  unsigned int BattSend = Battery * 1000;
  #ifdef DEBUG1
    Serial.println(Battery,3);
    Serial.println(BattSend);
  #endif  
  #ifdef DEBUG1
    String message = "TvB Cnt: " + String(counter) + ";" + String(BattSend) + ";" + String(BMP_T) + ";" + String(BMP_P) + ";" + String(HTU_t) + ";" + String(HTU_h);
  #endif
  #ifndef DEBUG1
    String message = String(counter) + ";" + String(BattSend) + ";" + String(BMP_T) + ";" + String(BMP_P) + ";" + String(HTU_t) + ";" + String(HTU_h);
  #endif  
  
  // #ifdevHEXsend
  //
  // after being bashed several times here should a routine for converting Stringdata to compressed HEXvalue be implemented
  //
  
  message.getBytes(buffer, message.length()+1);
  counter++;
   // Show TX channel (channel numbers are local to LMIC)
     #ifdef DEBUG1
       Serial.print("txCnhl: ");
       Serial.println(LMIC.txChnl);
       Serial.println("Sending: "+message);
     #endif 
    blink_x(blue, 1);
  LMIC_setTxData2(1, (uint8_t*) buffer, message.length() , 0);
}


void blink_x(int color, int numbers)  {          // colors:  red , blue
  for (int x=0; x!=numbers; x++) {
    digitalWrite(color, HIGH);
    delay(150);
    digitalWrite(color, LOW);
    delay(150);
   }
}


float Batt_Voltage(float R1, float R2) {
  // Measure voltage in "x.xx" volts
  analogRead(VoltagePin);
  delay(100);
  int sensorValue = analogRead(VoltagePin);               // A4
  float voltage1 = (sensorValue * 1.1) / 1000;
  float voltage2 = voltage1 * ((R1/R2) + 1);
  return voltage2;
}


void do_sleep() {
int var = 0;
  #ifdef DEBUG1
    Serial.print("sleeping ");
  #endif  
  while(var < NumberOf8Sec){            //  here 37 x 8 seconds = about 5 minutes
    // do something n times
    #ifdef DEBUG1
      delay(50); Serial.print(var); Serial.print(" "); delay(50);
    #endif  
    delay(50);                          // had problems without. no idea why
    var++; 
    // put the processor to sleep for 8 seconds
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
}


void readSensors() {
  digitalWrite(power, HIGH);
  delay(1000);                    // probably more needed ???

  if ( !BMPinit ) {
  HTU_h = 0.0;                    // in case of BMP180 could not be initialized
  HTU_t = 0.0;                    // set all values, except of the battey measurement,
  BMP_P = 0.0;                    // to zero, indicating an error
  BMP_T = 0.0;                    //
  }
  else{
  HTU_h = myHumidity.readHumidity();
  HTU_t = myHumidity.readTemperature();
  // --------------------------------------------------------------

  status = pressure.startTemperature();
  if (status != 0)  {
    // Wait for the measurement to complete:
    delay(status);
    status = pressure.getTemperature(BMP_T);
    if (status != 0)   {
      status = pressure.startPressure(3);
      if (status != 0)   {
        delay(status);
        status = pressure.getPressure(BMP_P, BMP_T);
        if (status != 0)   {
        }
        else {
          BMP_P = 0;
          #ifdef DEBUG1
            Serial.println("error retrieving pressure measurement\n");
          #endif  
        }
      }
      else {
        BMP_P = 0;
        #ifdef DEBUG1
          Serial.println("error starting pressure measurement\n");
        #endif  
      }
    }
    else {
      BMP_T = 0;
      #ifdef DEBUG1
        Serial.println("error retrieving temperature measurement\n");
      #endif
    }
  }
  else {
    BMP_T = 0;
    #ifdef DEBUG1
      Serial.println("error starting temperature measurement\n");
    #endif  
  }
  }
    digitalWrite(power, LOW);
}

//----------------------------------------------------------------------------------------
void setup() {
// initialize the digital pin as an output.
  pinMode(red, OUTPUT);
  pinMode(blue, OUTPUT);
  digitalWrite(red, LOW);  
  digitalWrite(blue, LOW);  

  pinMode(power, OUTPUT);
  digitalWrite(power, HIGH);                 // manage it also in the main loop!
  delay(5000);                                // 

   myHumidity.begin();
/*  
  if ( myHumidity.begin() ) {                 // sad, that the library doesn't respond with an error status
    HTUinit = true;
    Serial.println("HTU21D init success");
  }
  else {
    HTUinit = false;
    Serial.println("HTU21D init fails\n\n");
  }
*/  
  if ( pressure.begin() ) {
    BMPinit = true;
    #ifdef DEBUG1
      Serial.println("BMP180 init success");
    #endif  
  } 
  else  {
    #ifdef DEBUG1
      Serial.println("BMP180 init fail\n\n");
    #endif  
    BMPinit = false;
  }

  analogReference(INTERNAL1V1);           // don't miss that

  delay(2000);
  #ifdef DEBUG1
    Serial.begin(57600);
  #endif
  
  blink_x(red, 2);
  blink_x(blue, 2);

  #ifdef DEBUG1
    Serial.print("DeviceAddr: ");
    Serial.println(DEVADDR, HEX);
    Serial.println(F("Starting with personalized device ..."));
  #endif
  #ifndef CHANNEL0
    #ifdef DEBUG1
      Serial.print(F("all channels, "));
    #endif  
  #endif  
  #ifdef DEBUG1
     Serial.print(NumberOf8Sec*8/60);
     Serial.println(F(" minute interval"));
  #endif

  
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);

  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // restrict to channel 0 * aus einer mail von Thomas
    #ifdef CHANNEL0
      LMIC_disableChannel(1);
      LMIC_disableChannel(2);
      LMIC_disableChannel(3);
      LMIC_disableChannel(4);
      LMIC_disableChannel(5);
      LMIC_disableChannel(6);
      LMIC_disableChannel(7);
      LMIC_disableChannel(8);
      #ifdef DEBUG1
        Serial.println("---> ONLY channel 0 is active");
      #endif  
    #endif
    #ifdef DEBUG1
      Serial.println("----------------------------------");
      Serial.println();
     #endif
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.

  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

  // Start job
  do_send(&sendjob);
}


//----------------------------------------------------------------------------------------
void loop() {
  
  os_runloop_once();

}
