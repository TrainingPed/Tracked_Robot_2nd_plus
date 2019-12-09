#define trigPin01 12
#define echoPin01 13

void setup() 
{
  Serial.begin (9600);
  pinMode(trigPin01, OUTPUT);
  pinMode(echoPin01, INPUT);

}

void loop() 
{
  if(Serial.available()>0)
  {
    int duration01=0, distance01=0;
    
    
    digitalWrite(trigPin01, HIGH);
    delayMicroseconds(100);
    digitalWrite(trigPin01, LOW);
    duration01 = pulseIn(echoPin01, HIGH);
    
    distance01 = (int)(duration01/2) / 29.1;
    Serial.read();
    Serial.print(distance01);
    Serial.print(";");
    //delay(500);
  }
}
