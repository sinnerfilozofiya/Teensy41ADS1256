#include <SPI.h>



//(other stuff for getting the ADS1526 to work is in the next tab
// Updated pin definitions for Teensy 4.1
#define ADS_RST_PIN    8  //ADS1256 reset pin (same as Teensy 3.1)
#define ADS_RDY_PIN    22 //ADS1256 data ready (same as Teensy 3.1) 
#define ADS_CS_PIN     10 //ADS1256 chip select (Teensy 4.1 default SPI CS)

/* 
    Teensy 4.1 SPI0 pins:
    CLK  - pin 13 (SCK)
    DIN  - pin 11 (MOSI)
    DOUT - pin 12 (MISO)
    CS   - pin 10 (default CS, but we define our own above)
*/


//put the ADC constants here

double resolution = 8388608.; //2^23-1

//this needs to match the setting in the ADC init function in the library tab
double Gain = 64.; //be sure to have a period 

double vRef = 5.0; //reference voltage

//we'll calculate this in setup
double bitToVolt = 0.;

void setup() {
  delay(1000);
  // Teensy 4.1 supports higher baud rates than 3.1
  Serial.begin(115200);
  Serial.println("booting");
  //initialize the ADS
  pinMode(ADS_CS_PIN, OUTPUT);

  pinMode(ADS_RDY_PIN, INPUT);
  pinMode(ADS_RST_PIN, OUTPUT);

  SPI.begin();

  initADS();
  Serial.println("done init");

  //determine the conversion factor
    //do some calculations for the constants
  bitToVolt = resolution*Gain/vRef;
}

int32_t val1;  // Load cell 1 (AIN0-AIN1)
int32_t val2;  // Load cell 2 (AIN2-AIN3)
int32_t val3;  // Load cell 3 (AIN4-AIN5)
int32_t val4;  // Load cell 4 (AIN6-AIN7)

void loop() {
  // Read all 4 load cells
  read_four_values();
  
  // Print all 4 load cell values
  Serial.print("LC1: ");
  Serial.print(val1);
  Serial.print("\tLC2: ");
  Serial.print(val2);
  Serial.print("\tLC3: ");
  Serial.print(val3);
  Serial.print("\tLC4: ");
  Serial.println(val4);

//
//double value = 0;
//for (int i = 0; i <10; i++){
//  value += read_Value();
//}
//
//value /= 10.;
//value /= bitToVolt;
//
//Serial.println(value,11);
}
