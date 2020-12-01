/*
 * 
 * rights for PID example, on which pid control here is based, belongs to rightful owner http://www.electronoobs.com/eng_arduino_tut24_code3.php
 * temperature reading based on OneWire -> DS18x20_Temperature 
 * 
 */
 
#include <OneWire.h>

OneWire  ds(8);

#define enA 9
#define enB 10
#define in1 6
#define in2 5
#define in3 4
#define in4 3

const int waterSensor = A2; 
const int groundMoistureSensor = A6;
const int buzzer = 2; 
int waterLevel;
int waterCounter = 0;
int moistureLevel;
bool canWaterPumpWork = false;
bool isWaterPumpWorking = false;

const float groundTemperature = 27.0;

float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;

int kp = 90;   int ki = 30;   int kd = 80; //do sprawdzenia
int PID_p = 0;    int PID_i = 0;    int PID_d = 0;

void setup() {
  
  Serial.begin(9600); 

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  pinMode(waterSensor, INPUT);
  pinMode(groundMoistureSensor, INPUT);
  pinMode(buzzer, OUTPUT);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 100);
}

void loop() {
  /*
   * READ SENSORS
   */
  moistureLevel = analogRead(groundMoistureSensor);
  waterLevel=analogRead(waterSensor); 
  if(waterLevel < 20 && waterCounter < 3){
    
    digitalWrite(buzzer, HIGH);
    delay(100);
    digitalWrite(buzzer, LOW);
    waterCounter++;

    canWaterPumpWork = false;
  }

  if(waterLevel > 20){
    canWaterPumpWork = true;
    waterCounter = 0;
  }

  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius;
  
  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }
  
  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }
 
  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  Serial.println();

  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  } 
 
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        
  
  delay(1000);     
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         
 
  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();
 
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; 
    if (data[7] == 0x10) {
      
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
  
    if (cfg == 0x00) raw = raw & ~7; 
    else if (cfg == 0x20) raw = raw & ~3; 
    else if (cfg == 0x40) raw = raw & ~1; 
    
  }
  celsius = (float)raw / 16.0;

  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.print(" Celsius, ");



  Serial.print("Moisture Sensor Value:");
  Serial.println(moistureLevel);
  Serial.print("Water Level Sensor Value:");
  Serial.println(waterLevel);
  
  /*
   * 
   * ACTUATORS
   * 
   */

   /*
    * WATER PUMP
    */
    
  if(moistureLevel > 700 && canWaterPumpWork == true)
  {
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  isWaterPumpWorking = true;
  }
  
  if(moistureLevel < 350 && isWaterPumpWorking == true)
  {
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  isWaterPumpWorking = false;
  }
  
  /*
   *HEATERS 
   */
   PID_error = groundTemperature - celsius;


   PID_p = 0.01*kp * PID_error;

   PID_i = 0.01*PID_i + (ki * PID_error);
  


   timePrev = Time;                            
   Time = millis();                            
   elapsedTime = (Time - timePrev) / 1000; 

   PID_d = 0.01*kd*((PID_error - previous_error)/elapsedTime);
 
   PID_value = PID_p + PID_i + PID_d;


   if(PID_value < 0)
   {    PID_value = 0;    }
   if(PID_value > 255)  
   {    PID_value = 255;  }
   
   Serial.print("Error:");
   Serial.println(PID_error);
   Serial.print("PID Value:");
   Serial.println(PID_value);
   
   analogWrite(enA,PID_value);
   previous_error = PID_error;     
   
  delay(1000);
}
