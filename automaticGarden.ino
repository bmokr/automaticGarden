const int waterSensor = A2; 
const int groundMoistureSensor = A6;
const int buzzer = 2; 
int waterLevel;
int moistureLevel;
int canWaterPumpWork = 0;

void setup() {
  Serial.begin(9600); 
  pinMode(waterSensor, INPUT);
  pinMode(groundMoistureSensor, INPUT);
  pinMode(buzzer, OUTPUT);
}

void loop() {
  moistureLevel = analogRead(groundMoistureSensor);
  waterLevel=analogRead(waterSensor); 
  if(waterLevel < 20){
  /*  digitalWrite(buzzer, HIGH);
    delay(100);
    digitalWrite(buzzer, LOW);
    delay(1000);*/
    
    canWaterPumpWork = 0;
  }

  if(waterLevel > 20){
    canWaterPumpWork = 1;
  }
 /* if(humidityLevel < ? && canWaterPumpWork == 1)
  {
    //pompa sie zalacza z pid
  }*/
  Serial.print("Moisture Sensor Value:");
  Serial.println(moistureLevel);
  Serial.print("Water Level Sensor Value:");
  Serial.println(waterLevel);
  delay(1000);//delays 100ms
}
