/*
	MiniWx (c)ea3hmj 02/2023
*/
#include <Arduino.h>
#include "HardwareSerial.h"
#include <Adafruit_MLX90614.h>
#include <Adafruit_BME280.h>
#include <Preferences.h>
// Modbus server include
//#define LOG_LEVEL LOG_LEVEL_VERBOSE
//#include "src\Logging.h"
#include "src\ModbusServerRTU.h"
#define LED_BUILTIN 15

enum {     
	// The first register starts at address 0
	TSKY,			// R0
	TAMB,			// R1
	PRES,				
	HUMi,
	TSEN,				// R4
  TOTAL_REGS_SIZE   // total number of registers for function 3 and 16 share the same register array

};
uint16_t   holdingRegs[TOTAL_REGS_SIZE*2]; // function 3 and 16 register array
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_BME280 bme; // use I2C interface
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();
sensors_event_t temp_event, pressure_event, humidity_event;
Preferences preferences;
// Create a ModbusRTU server instance listening with 2000ms timeout
ModbusServerRTU MBserver(2000);

// FC03: worker do serve Modbus function code 0x03 (READ_HOLD_REGISTER)
ModbusMessage FC03(ModbusMessage request) {
  uint16_t address;           // requested register address
  uint16_t words;             // requested number of registers
  ModbusMessage response;     // response message to be sent back
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)

/*  bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);
  holdingRegs[TSKY]=(uint16_t)(mlx.readObjectTempC()*10);
  holdingRegs[TAMB]=(uint16_t)(mlx.readAmbientTempC()*10);
  holdingRegs[PRES]=(uint16_t)(pressure_event.pressure);
  holdingRegs[HUMi]=(uint16_t)(humidity_event.relative_humidity);
  holdingRegs[TSEN]=(uint16_t)(temp_event.temperature*10);
*/
  // get request values
  request.get(2, address);
  request.get(4, words);

  // Address and words valid? We assume 10 registers here for demo
  if ((address + words) <= TOTAL_REGS_SIZE*2) {
    // Looks okay. Set up message with serverID, FC and length of data
    response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));
    // Fill response with requested data
    for (uint16_t i = address; i < address + words; ++i) {
      response.add(holdingRegs[i]);
    }
  } else {
    // No, either address or words are outside the limits. Set up error response.
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }
//  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // turn the LED off by making the voltage LOW

  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW

  return response;
}


// Worker function function code 0x06
ModbusMessage FC06(ModbusMessage request) {
  uint16_t addr = 0;        // Start address to read
  uint16_t value = 0;       // New value for register
  ModbusMessage response;
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  // Get addr and value from data array. Values are MSB-first, getValue() will convert to binary
  request.get(2, addr);
  request.get(4, value);
  // address valid?
  if (  addr < TOTAL_REGS_SIZE || addr >= TOTAL_REGS_SIZE*2) {
    // No. Return error response
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    return response;
  }
    // Fill in new value.
  holdingRegs[addr] = value;
  preferences.begin("wx", false);
  char tmp[5];
  sprintf(tmp,"R%d",addr);
  preferences.putUInt(tmp, value);
  // Close the Preferences
  preferences.end();
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  return request;
}
// Setup() - initialization happens here
void setup() {
  preferences.begin("wx", false);
  char tmp[5];
  // inicializamos los offset
  for (int i=0;i<TOTAL_REGS_SIZE;i++){
	  sprintf(tmp,"R%d",TOTAL_REGS_SIZE+i);
	  holdingRegs[TOTAL_REGS_SIZE+i]=preferences.getUInt(tmp, 0);
  }
  // Close the Preferences
  preferences.end();
  delay(1000);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);                      // wait for a second
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
 // Init Serial monitor
  Serial.begin(115200);
  //while (!Serial) {}
  Serial.println("__ OK __");

// define pins de i2c
	Wire.begin(13,14, (uint32_t)100000); 

  
  if (!mlx.begin()){//MLX90614_I2CADDR, &I2Ctwo)){
	  Serial.println("MLX90614 no found!!");  
	}else{
    Serial.println("MLX90614 found!!");  
    Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC());
    Serial.print("°C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("°C");
    Serial.println();    
	}
  delay(250);                      // wait for a second
  if (!bme.begin(0x76)) {
    Serial.println(F("BME280 no found !!"));
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
  }else{
    Serial.println(F("BME280 found !!"));
    bme_temp->getEvent(&temp_event);
    bme_pressure->getEvent(&pressure_event);
    bme_humidity->getEvent(&humidity_event);
    Serial.print("Presure = "); Serial.print(pressure_event.pressure);
    Serial.print("mBar\tHumidity = "); Serial.print(humidity_event.relative_humidity);
    Serial.println("%");
  }
  
// Init Serial2 connected to the RTU Modbus
// (Fill in your data here!)
  RTUutils::prepareHardwareSerial(Serial1);
  Serial1.begin(57600, SERIAL_8N1, GPIO_NUM_16, GPIO_NUM_17);

  // Define and start RTU server
    MBserver.registerWorker(3, READ_HOLD_REGISTER, &FC03);      // FC=03 for serverID=1
   // MBserver.registerWorker(1, READ_INPUT_REGISTER, &FC04);     // FC=04 for serverID=1
    MBserver.registerWorker(3, WRITE_HOLD_REGISTER, &FC06);     // FC=06 for serverID=1
 
// Start ModbusRTU background task
  MBserver.begin(Serial1);

}

// loop() - nothing done here today!
void loop() {
   bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);
  holdingRegs[TSKY]=(uint16_t)(mlx.readObjectTempC()*10);
  holdingRegs[TAMB]=(uint16_t)(mlx.readAmbientTempC()*10);
  holdingRegs[PRES]=(uint16_t)(pressure_event.pressure);
  holdingRegs[HUMi]=(uint16_t)(humidity_event.relative_humidity);
  holdingRegs[TSEN]=(uint16_t)(temp_event.temperature*10);
  delay(250);
}
