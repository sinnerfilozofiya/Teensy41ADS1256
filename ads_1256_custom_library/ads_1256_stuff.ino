//my code
//note...this is written for TEENSY meaning I am using DigitalWriteFast to speed things up.
//Updated for Teensy 4.1: CS pin is now defined as ADS_CS_PIN (pin 10) in the main file
//see this post about how to use: https://forum.pjrc.com/threads/24573-Speed-of-digitalRead-and-digitalWrite-with-Teensy3-0

//built up on the work of:
//https://github.com/Flydroid/ADS12xx-Library
//https://gist.github.com/dariosalvi78/f2e990b4317199d235bbf5963c3486ae
//https://github.com/adienakhmad/ADS1256

// Global variables
volatile int DRDY_state = HIGH;


void initADS() {
  attachInterrupt(ADS_RDY_PIN, DRDY_Interuppt, FALLING);

  digitalWrite(ADS_RST_PIN, LOW);
  delay(10); // LOW at least 4 clock cycles of onboard clock. 100 microsecons is enough
  digitalWrite(ADS_RST_PIN, HIGH); // now reset to deafult values

  delay(1000);

  //now reset the ADS
  Reset();

  //let the system power up and stabilize (datasheet pg 24)
  delay(2000);
  //this enables the buffer which gets us more accurate voltage readings
 // SetRegisterValue(STATUS,B00110010);

  Serial.println(GetRegisterValue(STATUS));

  //next set the mux register
  //we are only trying to read differential values from pins 0 and 1. your needs may vary.
  //this is the default setting so we can just reset it
  SetRegisterValue(MUX,MUX_RESET); //set the mux register
   //B00001000 for single ended measurement

  //now set the ADCON register
  //set the PGA to 64x
  //you need to adjust the constants for the other ones according to datasheet pg 31 if you need other values
  SetRegisterValue(ADCON, PGA_64); //set the adcon register

  //next set the data rate - use higher rate for faster settling
  SetRegisterValue(DRATE, DR_7500); //set the drate register to 7500 SPS for faster settling

  //we're going to ignore the GPIO for now...

  //lastly, we need to calibrate the system

  //let it settle
  delay(2000);

  //then do calibration
  SendCMD(SELFCAL); //send the calibration command

  //then print out the values
  delay(5);

  Serial.print("OFC0: ");
  Serial.println(GetRegisterValue(OFC0));
  Serial.print("OFC1: ");
  Serial.println(GetRegisterValue(OFC1));
  Serial.print("OFC2: ");
  Serial.println(GetRegisterValue(OFC2));
  Serial.print("FSC0: ");
  Serial.println(GetRegisterValue(FSC0));
  Serial.print("FSC1: ");
  Serial.println(GetRegisterValue(FSC1));
  Serial.print("FSC2: ");
  Serial.println(GetRegisterValue(FSC2));
}

//function to read a value
//this assumes that we are not changing the mux action
int32_t read_Value() {
  int32_t adc_val = 0;
  waitforDRDY(); // Wait until DRDY is LOW
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  digitalWriteFast(ADS_CS_PIN, LOW); //Pull SS Low to Enable Communications with ADS1256
  //delayMicroseconds(5); // RD: Wait 25ns for ADC12xx to get ready
  SPI.transfer(RDATA); //Issue RDATA
  delayMicroseconds(7);
  adc_val |= SPI.transfer(NOP);
  //delayMicroseconds(10);
  adc_val <<= 8;
  adc_val |= SPI.transfer(NOP);
  //delayMicroseconds(10);
  adc_val <<= 8;
  adc_val |= SPI.transfer(NOP);
  //delayMicroseconds(5);
  digitalWriteFast(ADS_CS_PIN, HIGH);
  SPI.endTransaction();

  if (adc_val > 0x7fffff) { //if MSB == 1
    adc_val = adc_val - 16777216; //do 2's complement, keep the sign this time!
  }

  return adc_val;
}

//this function will go through the process of resetting the system and cycling through the inputs
//it assumes that whatever was called before left the mux at the default (0,1)
//this means it will treat the first value it receives as from (0,1)
//and the second from (2,3).

void read_two_values() {
  //datasheet page 21 at the bottom gives the timing
  int32_t adc_val1 = 0;
  int32_t adc_val2 = 0;

  waitforDRDY(); // Wait until DRDY is LOW
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  digitalWriteFast(ADS_CS_PIN, LOW); //Pull SS Low to Enable Communications with ADS1256
  //delayMicroseconds(5); // RD: Wait 25ns for ADC12xx to get ready

  //now change the mux register
  SPI.transfer(WREG | MUX); // send 1st command byte, address of the register
  SPI.transfer(0x00);   // send 2nd command byte, write only one register
  SPI.transfer(0x23);     //pins registers 2 and 3

  //now we need to sync
  //need to delay by 4x SPI clock = 2.35 uS (t1)
  //to be safe 5 uS
  delayMicroseconds(2);
  SPI.transfer(SYNC);

  //again delay by t1
  delayMicroseconds(5);
  //send wakeup
  SPI.transfer(WAKEUP);

  //then delay one more time by t1 before rdata
  delayMicroseconds(1);

  SPI.transfer(RDATA); //Issue RDATA
  delayMicroseconds(7);

  //This is the reading in the Data register from whatever the mux was set to the last
  //time this function was called. By default, it is configured to leave that value at 0
  adc_val1 |= SPI.transfer(NOP);
  //delayMicroseconds(10);
  adc_val1 <<= 8;
  adc_val1 |= SPI.transfer(NOP);
  //delayMicroseconds(10);
  adc_val1 <<= 8;
  adc_val1 |= SPI.transfer(NOP);
  //delayMicroseconds(5);

  //now wait for the next dataready
  waitforDRDY(); // Wait until DRDY is LOW
  //delayMicroseconds(5);

  //now change the mux register back to 0 so we left things how we found them
  SPI.transfer(WREG | MUX); // send 1st command byte, address of the register
  SPI.transfer(0x00);   // send 2nd command byte, write only one register
  SPI.transfer(MUX_RESET);     //pins registers 2 and 3

  //now we need to sync
  //need to delay by 4x SPI clock = 2.35 uS (t1)
  //to be safe 5 uS
  delayMicroseconds(2);
  SPI.transfer(SYNC);

  //again delay by t1
  delayMicroseconds(5);
  //send wakeup
  SPI.transfer(WAKEUP);

  //then delay one more time by t1 before rdata
  delayMicroseconds(1);

  SPI.transfer(RDATA); //Issue RDATA
  delayMicroseconds(7);
  //this should now be the value from the mux change we just did (0,1 to 2,3)
  adc_val2 |= SPI.transfer(NOP);
  //delayMicroseconds(10);
  adc_val2 <<= 8;
  adc_val2 |= SPI.transfer(NOP);
  //delayMicroseconds(10);
  adc_val2 <<= 8;
  adc_val2 |= SPI.transfer(NOP);
  //delayMicroseconds(5);
  //this is the value for the

  digitalWriteFast(ADS_CS_PIN, HIGH);
  SPI.endTransaction();

  if (adc_val1 > 0x7fffff) { //if MSB == 1
    adc_val1 = adc_val1 - 16777216; //do 2's complement, keep the sign this time!
  }

  if (adc_val2 > 0x7fffff) { //if MSB == 1
    adc_val2 = adc_val2 - 16777216; //do 2's complement, keep the sign this time!
  }

  val1 = adc_val1;

  val2 = adc_val2;
}

void read_three_values() {
  //datasheet page 21 at the bottom gives the timing
  int32_t adc_val1 = 0;
  int32_t adc_val2 = 0;
  int32_t adc_val3 = 0;

  waitforDRDY(); // Wait until DRDY is LOW
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  digitalWriteFast(ADS_CS_PIN, LOW); //Pull SS Low to Enable Communications with ADS1256
  //delayMicroseconds(5); // RD: Wait 25ns for ADC12xx to get ready

  //now change the mux register
  SPI.transfer(WREG | MUX); // send 1st command byte, address of the register
  SPI.transfer(0x00);   // send 2nd command byte, write only one register
  SPI.transfer(0x23);     //pins registers 2 and 3

  //now we need to sync
  //need to delay by 4x SPI clock = 2.35 uS (t1)
  //to be safe 5 uS
  delayMicroseconds(2);
  SPI.transfer(SYNC);

  //again delay by t1
  delayMicroseconds(5);
  //send wakeup
  SPI.transfer(WAKEUP);

  //then delay one more time by t1 before rdata
  delayMicroseconds(1);

  SPI.transfer(RDATA); //Issue RDATA
  delayMicroseconds(7);

  //This is the reading in the Data register from whatever the mux was set to the last
  //time this function was called. By default, it is configured to leave that value at 0
  adc_val1 |= SPI.transfer(NOP);
  //delayMicroseconds(10);
  adc_val1 <<= 8;
  adc_val1 |= SPI.transfer(NOP);
  //delayMicroseconds(10);
  adc_val1 <<= 8;
  adc_val1 |= SPI.transfer(NOP);
  //delayMicroseconds(5);

  //now wait for the next dataready
  waitforDRDY(); // Wait until DRDY is LOW
  //delayMicroseconds(5);

  //now change the mux register
  SPI.transfer(WREG | MUX); // send 1st command byte, address of the register
  SPI.transfer(0x00);   // send 2nd command byte, write only one register
  SPI.transfer(0x45);     //pins registers 4 and 5

  //now we need to sync
  //need to delay by 4x SPI clock = 2.35 uS (t1)
  //to be safe 5 uS
  delayMicroseconds(2);
  SPI.transfer(SYNC);

  //again delay by t11
  delayMicroseconds(5);
  //send wakeup
  SPI.transfer(WAKEUP);

  //then delay one more time by t1 before rdata
  delayMicroseconds(1);

  SPI.transfer(RDATA); //Issue RDATA
  delayMicroseconds(7);

  //this is the reading from pins 2,3
  adc_val2 |= SPI.transfer(NOP);
  //delayMicroseconds(10);
  adc_val2 <<= 8;
  adc_val2 |= SPI.transfer(NOP);
  //delayMicroseconds(10);
  adc_val2 <<= 8;
  adc_val2 |= SPI.transfer(NOP);
  //delayMicroseconds(5);

  //now wait for the next dataready
  waitforDRDY(); // Wait until DRDY is LOW
  //delayMicroseconds(5);

  //now change the mux register back to 0 so we left things how we found them
  SPI.transfer(WREG | MUX); // send 1st command byte, address of the register
  SPI.transfer(0x00);   // send 2nd command byte, write only one register
  SPI.transfer(MUX_RESET);     //pins registers 2 and 3

  //now we need to sync
  //need to delay by 4x SPI clock = 2.35 uS (t1)
  //to be safe 5 uS
  delayMicroseconds(2);
  SPI.transfer(SYNC);

  //again delay by t1
  delayMicroseconds(5);
  //send wakeup
  SPI.transfer(WAKEUP);

  //then delay one more time by t1 before rdata
  delayMicroseconds(1);

  SPI.transfer(RDATA); //Issue RDATA
  delayMicroseconds(7);
  //this should now be the value from the mux change we just did (2,3 to 4,5)
  adc_val3 |= SPI.transfer(NOP);
  //delayMicroseconds(10);
  adc_val3 <<= 8;
  adc_val3 |= SPI.transfer(NOP);
  //delayMicroseconds(10);
  adc_val3 <<= 8;
  adc_val3 |= SPI.transfer(NOP);
 // delayMicroseconds(5);
  //this is the value for the

  digitalWriteFast(ADS_CS_PIN, HIGH);
  SPI.endTransaction();

  if (adc_val1 > 0x7fffff) { //if MSB == 1
    adc_val1 = adc_val1 - 16777216; //do 2's complement, keep the sign this time!
  }

  if (adc_val2 > 0x7fffff) { //if MSB == 1
    adc_val2 = adc_val2 - 16777216; //do 2's complement, keep the sign this time!
  }

  if (adc_val3 > 0x7fffff) { //if MSB == 1
    adc_val3 = adc_val3 - 16777216; //do 2's complement, keep the sign this time!
  }


  val1 = adc_val1;

  val2 = adc_val2;

  val3 = adc_val3;
}

void read_four_values() {
  //datasheet page 21 at the bottom gives the timing
  int32_t adc_val1 = 0;
  int32_t adc_val2 = 0;
  int32_t adc_val3 = 0;
  int32_t adc_val4 = 0;

  waitforDRDY(); // Wait until DRDY is LOW
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  digitalWriteFast(ADS_CS_PIN, LOW); //Pull SS Low to Enable Communications with ADS1256

  //now change the mux register to read channels 2,3
  SPI.transfer(WREG | MUX); // send 1st command byte, address of the register
  SPI.transfer(0x00);   // send 2nd command byte, write only one register
  SPI.transfer(0x23);     //pins registers 2 and 3

  //now we need to sync
  delayMicroseconds(2);
  SPI.transfer(SYNC);
  delayMicroseconds(5);
  SPI.transfer(WAKEUP);
  delayMicroseconds(1);

  SPI.transfer(RDATA); //Issue RDATA
  delayMicroseconds(7);

  //This is the reading from channels 0,1 (from previous mux setting)
  adc_val1 |= SPI.transfer(NOP);
  adc_val1 <<= 8;
  adc_val1 |= SPI.transfer(NOP);
  adc_val1 <<= 8;
  adc_val1 |= SPI.transfer(NOP);

  //now wait for the next dataready
  waitforDRDY(); // Wait until DRDY is LOW

  //now change the mux register to read channels 4,5
  SPI.transfer(WREG | MUX); // send 1st command byte, address of the register
  SPI.transfer(0x00);   // send 2nd command byte, write only one register
  SPI.transfer(0x45);     //pins registers 4 and 5

  delayMicroseconds(2);
  SPI.transfer(SYNC);
  delayMicroseconds(5);
  SPI.transfer(WAKEUP);
  delayMicroseconds(1);

  SPI.transfer(RDATA); //Issue RDATA
  delayMicroseconds(7);

  //this is the reading from pins 2,3
  adc_val2 |= SPI.transfer(NOP);
  adc_val2 <<= 8;
  adc_val2 |= SPI.transfer(NOP);
  adc_val2 <<= 8;
  adc_val2 |= SPI.transfer(NOP);

  //now wait for the next dataready
  waitforDRDY(); // Wait until DRDY is LOW

  //now change the mux register to read channels 6,7
  SPI.transfer(WREG | MUX); // send 1st command byte, address of the register
  SPI.transfer(0x00);   // send 2nd command byte, write only one register
  SPI.transfer(0x67);     //pins registers 6 and 7

  delayMicroseconds(2);
  SPI.transfer(SYNC);
  delayMicroseconds(5);
  SPI.transfer(WAKEUP);
  delayMicroseconds(1);

  SPI.transfer(RDATA); //Issue RDATA
  delayMicroseconds(7);

  //this is the reading from pins 4,5
  adc_val3 |= SPI.transfer(NOP);
  adc_val3 <<= 8;
  adc_val3 |= SPI.transfer(NOP);
  adc_val3 <<= 8;
  adc_val3 |= SPI.transfer(NOP);

  //now wait for the next dataready
  waitforDRDY(); // Wait until DRDY is LOW

  //now change the mux register back to 0,1 so we left things how we found them
  SPI.transfer(WREG | MUX); // send 1st command byte, address of the register
  SPI.transfer(0x00);   // send 2nd command byte, write only one register
  SPI.transfer(MUX_RESET);     //pins registers 0 and 1

  delayMicroseconds(2);
  SPI.transfer(SYNC);
  delayMicroseconds(5);
  SPI.transfer(WAKEUP);
  delayMicroseconds(1);

  SPI.transfer(RDATA); //Issue RDATA
  delayMicroseconds(7);

  //this should now be the value from pins 6,7
  adc_val4 |= SPI.transfer(NOP);
  adc_val4 <<= 8;
  adc_val4 |= SPI.transfer(NOP);
  adc_val4 <<= 8;
  adc_val4 |= SPI.transfer(NOP);

  digitalWriteFast(ADS_CS_PIN, HIGH);
  SPI.endTransaction();

  // Convert to signed values (2's complement)
  if (adc_val1 > 0x7fffff) { //if MSB == 1
    adc_val1 = adc_val1 - 16777216; //do 2's complement, keep the sign this time!
  }

  if (adc_val2 > 0x7fffff) { //if MSB == 1
    adc_val2 = adc_val2 - 16777216; //do 2's complement, keep the sign this time!
  }

  if (adc_val3 > 0x7fffff) { //if MSB == 1
    adc_val3 = adc_val3 - 16777216; //do 2's complement, keep the sign this time!
  }

  if (adc_val4 > 0x7fffff) { //if MSB == 1
    adc_val4 = adc_val4 - 16777216; //do 2's complement, keep the sign this time!
  }

  // Store results in global variables
  val1 = adc_val1;
  val2 = adc_val2;
  val3 = adc_val3;
  val4 = adc_val4;
}

// Optimized function for faster single-channel reads at high SPS
int32_t read_single_channel(uint8_t mux_setting) {
  int32_t adc_val = 0;
  
  waitforDRDY(); // Wait until DRDY is LOW
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  digitalWriteFast(ADS_CS_PIN, LOW);
  
  // Set MUX register
  SPI.transfer(WREG | MUX);
  SPI.transfer(0x00);
  SPI.transfer(mux_setting);
  
  // Minimal sync sequence for speed
  delayMicroseconds(1);
  SPI.transfer(SYNC);
  delayMicroseconds(3);
  SPI.transfer(WAKEUP);
  delayMicroseconds(1);
  
  // Read data
  SPI.transfer(RDATA);
  delayMicroseconds(5);
  adc_val |= SPI.transfer(NOP);
  adc_val <<= 8;
  adc_val |= SPI.transfer(NOP);
  adc_val <<= 8;
  adc_val |= SPI.transfer(NOP);
  
  digitalWriteFast(ADS_CS_PIN, HIGH);
  SPI.endTransaction();
  
  // Convert to signed
  if (adc_val > 0x7fffff) {
    adc_val = adc_val - 16777216;
  }
  
  return adc_val;
}

// Ultra-fast single channel read for 1000 SPS
int32_t read_single_channel_ultra_fast(uint8_t mux_setting) {
  int32_t adc_val = 0;
  
  // Start SPI transaction
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  digitalWriteFast(ADS_CS_PIN, LOW);
  
  // Set MUX register
  SPI.transfer(WREG | MUX);
  SPI.transfer(0x00);
  SPI.transfer(mux_setting);
  
  // Minimal sync - just enough for ADS1256
  SPI.transfer(SYNC);
  SPI.transfer(WAKEUP);
  
  // Wait for DRDY without ending transaction
  digitalWriteFast(ADS_CS_PIN, HIGH);
  waitforDRDY();
  digitalWriteFast(ADS_CS_PIN, LOW);
  
  // Read data immediately
  SPI.transfer(RDATA);
  delayMicroseconds(7);  // t6 delay
  adc_val |= SPI.transfer(NOP);
  adc_val <<= 8;
  adc_val |= SPI.transfer(NOP);
  adc_val <<= 8;
  adc_val |= SPI.transfer(NOP);
  
  digitalWriteFast(ADS_CS_PIN, HIGH);
  SPI.endTransaction();
  
  // Convert to signed
  if (adc_val > 0x7fffff) {
    adc_val = adc_val - 16777216;
  }
  
  return adc_val;
}

// State machine for non-blocking ADS1256 reading
// Unused non-blocking and round-robin code removed - using blocking read_four_values() instead

// Fast 4-channel read optimized for 1000 SPS
void read_four_values_fast() {
  // Read channels in sequence with minimal delays
  val1 = read_single_channel(0x01); // AIN0-AIN1
  val2 = read_single_channel(0x23); // AIN2-AIN3  
  val3 = read_single_channel(0x45); // AIN4-AIN5
  val4 = read_single_channel(0x67); // AIN6-AIN7
  
  // Reset to default MUX setting
  waitforDRDY();
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  digitalWriteFast(ADS_CS_PIN, LOW);
  SPI.transfer(WREG | MUX);
  SPI.transfer(0x00);
  SPI.transfer(MUX_RESET);
  digitalWriteFast(ADS_CS_PIN, HIGH);
  SPI.endTransaction();
}


//library files

void waitforDRDY() {
  while (DRDY_state) {
    continue;
  }
  noInterrupts();
  DRDY_state = HIGH;
  interrupts();
}

//Interrupt function
void DRDY_Interuppt() {
  DRDY_state = LOW;
}

long GetRegisterValue(uint8_t regAdress) {
  uint8_t bufr;
  digitalWriteFast(ADS_CS_PIN, LOW);
  delayMicroseconds(10);
  SPI.transfer(RREG | regAdress); // send 1st command byte, address of the register
  SPI.transfer(0x00);     // send 2nd command byte, read only one register
  delayMicroseconds(10);
  bufr = SPI.transfer(NOP); // read data of the register
  delayMicroseconds(10);
  digitalWriteFast(ADS_CS_PIN, HIGH);
  //digitalWrite(_START, LOW);
  SPI.endTransaction();
  return bufr;

}

void SendCMD(uint8_t cmd) {
  waitforDRDY();
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1)); // initialize SPI with 4Mhz clock, MSB first, SPI Mode0
  digitalWriteFast(ADS_CS_PIN, LOW);
  delayMicroseconds(10);
  SPI.transfer(cmd);
  delayMicroseconds(10);
  digitalWriteFast(ADS_CS_PIN, HIGH);
  SPI.endTransaction();
}

void Reset() {
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1)); // initialize SPI with  clock, MSB first, SPI Mode1
  digitalWriteFast(ADS_CS_PIN, LOW);
  delayMicroseconds(10);
  SPI.transfer(RESET); //Reset
  delay(2); //Minimum 0.6ms required for Reset to finish.
  SPI.transfer(SDATAC); //Issue SDATAC
  delayMicroseconds(100);
  digitalWriteFast(ADS_CS_PIN, HIGH);
  SPI.endTransaction();
}

void SetRegisterValue(uint8_t regAdress, uint8_t regValue) {

  uint8_t regValuePre = GetRegisterValue(regAdress);
  if (regValue != regValuePre) {
    //digitalWrite(_START, HIGH);
    delayMicroseconds(10);
    waitforDRDY();
    SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1)); // initialize SPI with SPI_SPEED, MSB first, SPI Mode1
    digitalWriteFast(ADS_CS_PIN, LOW);
    delayMicroseconds(10);
    SPI.transfer(WREG | regAdress); // send 1st command byte, address of the register
    SPI.transfer(0x00);   // send 2nd command byte, write only one register
    SPI.transfer(regValue);         // write data (1 Byte) for the register
    delayMicroseconds(10);
    digitalWriteFast(ADS_CS_PIN, HIGH);
    //digitalWrite(_START, LOW);
    if (regValue != GetRegisterValue(regAdress)) {   //Check if write was succesfull
      Serial.print("Write to Register 0x");
      Serial.print(regAdress, HEX);
      Serial.println(" failed!");
    }
    else {
      Serial.println("success");
    }
    SPI.endTransaction();

  }

}
