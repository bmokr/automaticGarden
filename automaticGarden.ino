const int waterSensor = A2; //sensor pin connected to analog pin A0
const int buzzer = 2; //buzzer to arduino pin D2
int waterLevel;
int canWaterPumpWork = 0;

void setup() {
  Serial.begin(9600); //sets the baud rate for data transfer in bits/second
  pinMode(waterSensor, INPUT);//the liquid level sensor will be an input to the arduino
  pinMode(buzzer, OUTPUT);
}

void loop() {
  waterLevel=analogRead(waterSensor) ; //arduino reads the value from the liquid level sensor
  if(waterLevel < 20){
    digitalWrite(buzzer, HIGH);
    delay(100);
    digitalWrite(buzzer, LOW);
    canWaterPumpWork = 0;
  }

  if(waterLevel > 0){
    canWaterPumpWork = 1;
  }
  Serial.println(waterLevel);//prints out liquid level sensor reading
  delay(100);//delays 100ms
}
