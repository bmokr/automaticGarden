#include <OneWire.h>

OneWire  ds(8);

#define enA 9
#define in1 6
#define in2 5
#define in3 4
#define in4 3

const int waterSensor = A2; 
const int groundMoistureSensor = A6;
const int buzzer = 2; 
int waterLevel;
int moistureLevel;
bool canWaterPumpWork = false;
bool isWaterPumpWorking = false;

const float groundTemperature = 60.0;

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

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, 0);
}

void loop() {
  /*
   * READ SENSORS
   */
  moistureLevel = analogRead(groundMoistureSensor);
  waterLevel=analogRead(waterSensor); 
  if(waterLevel < 20){
  /*  digitalWrite(buzzer, HIGH);
    delay(100);
    digitalWrite(buzzer, LOW);
    
    
    */
    
    canWaterPumpWork = false;
  }

  if(waterLevel > 20){
    canWaterPumpWork = true;
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

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
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
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad
 
  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();
 
  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
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
    
  if(moistureLevel > 600 && canWaterPumpWork == true)
  {
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  isWaterPumpWorking = true;
  delay(10000);
  }
  
  if(moistureLevel < 700 && isWaterPumpWorking == true)
  {
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  isWaterPumpWorking = false;
  }
  
  /*
   *HEATERS 
   */
   PID_error = groundTemperature - celsius;

   //Calculate the P value
   PID_p = 0.01*kp * PID_error;
   //Calculate the I value in a range on +-3
   PID_i = 0.01*PID_i + (ki * PID_error);
  

   //For derivative we need real time to calculate speed change rate
   timePrev = Time;                            // the previous time is stored before the actual time read
   Time = millis();                            // actual time read
   elapsedTime = (Time - timePrev) / 1000; 
   //Now we can calculate the D calue
   PID_d = 0.01*kd*((PID_error - previous_error)/elapsedTime);
   //Final total PID value is the sum of P + I + D
   PID_value = PID_p + PID_i + PID_d;

   //We define PWM range between 0 and 255
   if(PID_value < 0)
   {    PID_value = 0;    }
   if(PID_value > 255)  
   {    PID_value = 255;  }
   
   Serial.print("Error:");
   Serial.println(PID_error);
   Serial.print("PID Value:");
   Serial.println(PID_value);
   
   analogWrite(enA,PID_value);
   previous_error = PID_error;     //Remember to store the previous error for next loop.
  delay(1000);
}
