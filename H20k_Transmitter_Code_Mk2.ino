
/**
   H20k Transmitter Node Code by Joseph Sanchez
   based off of LoRa Weather Client by Shawn Hymel under License: Beerware
   Also using code from Atlas Scientific's https://github.com/Atlas-Scientific/Ezo_I2c_lib/blob/master/Examples/pH_EC_led_indicator/pH_EC_led_indicator.ino 
   as part of their library documentation for Ezo_i2c.h

   Transmits temperature, turbidity, and salinity data over raw
   LoRa radio with packet format
   | 1B From Addr | 1B To Addr | 2B Temp | 2B Turbidity | 2B Salinity |

   Note that values are scaled up by 10 and rounded to nearest integer before being
   sent. The server will need to scale received values by 1/10.
   This is to avoid sending full floating point values.

   HW:
   Adafruit Feather 328p 
   Adafruit Featherwing RFM95 915Mhz (NA)  ---------------------------------- hopefully will condense these two into a single 32u4 feather with rfm95 built in for next upgrade
   Atlas Scientific EC Ezo conductity sensor kit (board, probe, BNC carrier)
   DFRobot breakout for DS18S20 temperature sensor
   Analog Turbidity sensor --------------------------------------------------(5V problem piece)

*/

// Grab necessary libraries
#include <Wire.h>
#include <SPI.h>
#include <OneWire.h>
#include <Adafruit_Sensor.h>
#include <RH_RF95.h> //Radiohead
#include <Ezo_i2c.h>

#define DEBUG 1

// Parameters
const uint8_t LORA_NODE_ADDR = 0x01;    // This node's address, change with each sensor
const uint8_t LORA_SERVER_ADDR = 0x00;  // LoRa receiving address, constant for basestation
const int WAIT_TIME = 300;              // ms 30s normally, .3 for debugging
const int TX_BUF_SIZE = 8;              // Transmit buffer size
const float RFM_FREQ = 915.0;           // Frequency for RFM95W
const int RFM_TX_POWER = 17;            // 5..23 dBm, 13 dBm is default
const uint16_t CUTOFF_ADC = 505;        // Cutoff voltage for device (1.9V)
const uint8_t WAKEUP_CYCLES = 1;        // 8 Sleep cycles normally, 1 for debugging
const unsigned int response_delay = 1000; //how long we wait to receive a response from salinity sensor, in milliseconds

// Pins - may need to change for feather ------------------------------------------------------------------------------------------------------------------------------
const int turbpin = A7; //Pin only good for analog, perfect for this use
const int temppin = A0; //set to digital mode??
// SPI: inherently defined but for reference
// MOSI = 11
// MISO = 12
// SCK = 13
const int RFM_RST_PIN = 2;
const int RFM_INT_PIN = 3; //IRQ on breakout
const int RFM_CS_PIN = 4;

// unsure whether to implement this, don't need yet as no battery hooked up
//const int V_EN_PIN = 8;
//const int V_DIV_PIN = A0; ---------------------------------------------conflicts

// address for salinity sensor i2c
uint8_t address = 100;
//Constructor for Ezo Salinity Sensor
Ezo_board EC = Ezo_board(address, "EC");      //create an EC circuit object and name it "EC"

// Wakeup counter
uint8_t wakeup_count = WAKEUP_CYCLES;

// Instance of radio driver over SPI
RH_RF95 rfm(RFM_CS_PIN, RFM_INT_PIN);

// Setup onewire comms with Temp Sensor
//Temperature chip i/o
OneWire ds(temppin);

// Transmit buffer
uint8_t tx_buf[TX_BUF_SIZE];





void setup() {
#if DEBUG
  Serial.begin(9600);
#endif

  // Voltage divider enable
 // pinMode(V_EN_PIN, OUTPUT);

  // Disable ADC (must be before writing to PRR or ADC will be stuck on)
  ADCSRA = 0;

  // advanced sleeping, but i want i2c rn
  // Disable power to I2C, TIM2, TIM1, and ADC
  PRR = //(1 << PRTWI) |    // TWI (I2C) -----------actually leaving i2c on for salinity sensor, hurts a bit but gets buggy to flip on and off
        (1 << PRTIM2) |   // Timer/Counter2
        (1 << PRTIM1) |   // Timer/Counter1
        (1 << PRADC);     // ADC*/

        // Future improvement, set up comms, verify, save settings, turn off i2c, then reboot it from each sleep cycle
        // Nearly identical to how code does it with the ADC rn

  // Manually reset RFM95W
  pinMode(RFM_RST_PIN, OUTPUT);
  digitalWrite(RFM_RST_PIN, HIGH);
  delay(100);
  digitalWrite(RFM_RST_PIN, LOW);
  delay(10);
  digitalWrite(RFM_RST_PIN, HIGH);
  delay(10);

  // Initialize RFM95W
  if ( !rfm.init() ) {
#if DEBUG
    Serial.println("Could not initialize RFM95");
#endif
    while (1);
  }
#if DEBUG
  Serial.println("RFM95 initialized");
#endif

  // Set RFM95W frequency
  if ( !rfm.setFrequency(RFM_FREQ) ) {
#if DEBUG
    Serial.println("Could not set frequency on RFM95");
#endif
    while (1);
  }
#if DEBUG
  Serial.print("RFM95 frequency set to ");
  Serial.print(RFM_FREQ);
  Serial.println(" MHz");
#endif

  // Set RFM95W transmit power from PA_BOOST pin
  rfm.setTxPower(RFM_TX_POWER, false);

  //Start I2C comms
  Wire.begin(); 

  //Turn off LED on Salinity Sensor
  EC.send_cmd("L,0");

}






void loop() {

  // Check counter
  wakeup_count++;
  if ( wakeup_count > WAKEUP_CYCLES ) {
    wakeup_count = 0;

    // Turn power on to ADC
    PRR &= ~(1 << PRADC);

    // Enable ADC
    ADCSRA |= (1 << ADEN);

    // Discard first ADC reading for stability
  //  analogRead(V_DIV_PIN);

    // Only take measurements and transmit if over cutoff voltage --------------------------dont know if i want to implement this way
   // digitalWrite(V_EN_PIN, HIGH);
   // uint16_t v_batt = analogRead(V_DIV_PIN);
   // digitalWrite(V_EN_PIN, LOW);

  //  if ( v_batt > CUTOFF_ADC ) {

      // Take Reading from Temperature Sensor

      float temp = getTemp(); // in degrees Celsius

      // Take Reading from turbidity sensor
      int turbdraw = 2 * analogRead(turbpin);// read the input on turbpin, x2 bc of current voltage divider
      int turbd = turbdraw; // testing with this for now
     // float turbd = turbdraw*5.0/1024.0; // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):

      // do i do conversions here or on receiving node?? probably receiving for power efficiency
      // maybe shouldn't do so many conversions for this one, seems excessive since i don't have that much resolution ---------------------------------------------

      // Take Reading from Salinity sensor
      EC.send_read_cmd();
      delay(response_delay); // Wait long enough for sensor to take a reading
      receive_reading(EC);   //get the reading from the EC circuit
      
      
      #if DEBUG
        Serial.println(EC.get_last_received_reading());               //the command was successful, print the reading
      #endif

      float salt = EC.get_last_received_reading(); //-----------------------------------------------------------------------check formatting
      

      // Apply conversion factor to Turbidity and Salinity Data
      // Relevant after Calibration



 
      // Scale (x10) and round data to make it ints instead of floats for buffer size limits
      int16_t tempt = (int16_t)((temp * 10.0) + 0.5);
      int16_t turbdt = (int16_t)((turbd * 10.0) + 0.5);
      int16_t saltt = (int16_t)((salt * 10.0) + 0.5);

#if DEBUG
      Serial.print("Temperature: ");
      Serial.print(temp, 1);
      Serial.println(" C");
      Serial.print("Turbidity: ");
      Serial.print(turbd, 1);
      Serial.println("not calibrated");
      Serial.print("Salinity: ");
      Serial.print(salt, 1);
      Serial.println(" Unitsss???");
#endif

      // Stuff buffer
      tx_buf[0] = LORA_NODE_ADDR;     // From address (this node) [1 byte]
      tx_buf[1] = LORA_SERVER_ADDR;   // To address (server) [1 byte]
      tx_buf[2] = (0xff & tempt);     // Temperature [2 bytes] little-endian
      tx_buf[3] = (0xff & (tempt >> 8));
      tx_buf[4] = (0xff & turbdt);     // Humidity [2 bytes] little-endian
      tx_buf[5] = (0xff & (turbdt >> 8));
      tx_buf[6] = (0xff & saltt);     // Saltinity [2 bytes] little-endian
      tx_buf[7] = (0xff & (saltt >> 8));

#if DEBUG
      Serial.print("Sending buffer:");
      for ( int i = 0; i < TX_BUF_SIZE; i++) {
        Serial.print(" 0x");
        Serial.print(tx_buf[i], HEX);
      }
      Serial.println();
      Serial.println();
#endif

      // Send data to server
      rfm.send(tx_buf, TX_BUF_SIZE);
      rfm.waitPacketSent();
    }

    // Disable ADC (must be before writing to PRR or ADC will be stuck on)
    ADCSRA = 0;

    // Disable power to ADC
    PRR |= (1 << PRADC);

    // Put RFM95 to sleep
    rfm.sleep();
 // }

  // Put 328p to sleep
  goToSleep();
}





// Function Definitions

// Interrupt Service Routine (Watchdog Timer)
ISR(WDT_vect) {

  // Disable Watchdog Timer
  asm("wdr");                         // Reset WDT
  WDTCSR |= (1 << WDCE) | (1 << WDE); // Special operation to change WDT config
  WDTCSR = 0x00;                      // Turn off WDT
}

// Set the processor to power-down sleep mode
void goToSleep() {

  // Disable interrupts while we configure sleep
  asm("cli");

  // Configure Watchdog Timer
  uint8_t wdt_timeout = (1 << WDP3) | (1 << WDP0); // 8.0 s timeout
  asm("wdr");                           // Reset WDT
  WDTCSR |= (1 << WDCE) | (1 << WDE);   // Special operation to change WDT config
  WDTCSR = (1 << WDIE) | wdt_timeout;   // Enable WDT interrupts, set timeout

  // Sleep sequence (call right before sleeping)
  SMCR |= (1 << SM1); // Power-down sleep mode
  SMCR |= (1 << SE);  // Enable sleep

  // Re-enable interrupts and call sleep instruction
  asm("sei");         // Enable interrupts
  asm("sleep");       // Go to sleep

  // -> Wake up here <-

  // Disable sleeping as a precaution
  SMCR &= ~(1 << SE); // Disable sleep
}

// Onewire comms protocol for Temp Sensor
float getTemp() {
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
    //no more sensors on chain, reset search
    ds.reset_search();
    return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;
}
//Reading Function for Salinity sensor to handle error codes
void receive_reading(Ezo_board &Sensor) {               // function to decode the reading after the read command was issued

    Serial.print(Sensor.get_name()); Serial.print(": ");  // print the name of the circuit getting the reading

  Sensor.receive_read_cmd();                                //get the response data and put it into the [Sensor].reading variable if successful
    switch (Sensor.get_error()) {                         //switch case based on what the response code is.
      case Ezo_board::SUCCESS:
        Serial.print(Sensor.get_last_received_reading());               //the command was successful, print the reading
        break;

      case Ezo_board::FAIL:
        Serial.print("Failed ");                          //means the command has failed.
        break;

      case Ezo_board::NOT_READY:
        Serial.print("Pending ");                         //the command has not yet been finished calculating.
        break;

      case Ezo_board::NO_DATA:
        Serial.print("No Data ");                         //the sensor has no data to send.
        break;
  }
}
