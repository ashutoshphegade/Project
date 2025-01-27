const int VALUE= 0;
const int MOISTURE_LEVEL1 = 500; 
const int MOISTURE_LEVEL2 = 800;

void setup()
{
Serial.begin(9600);
 pinMode(VALUE,INPUT);
 pinMode(9,OUTPUT);
 pinMode(10,OUTPUT);
 pinMode(11,OUTPUT);
 pinMode(3,OUTPUT);
 pinMode(2,OUTPUT); 
}

void loop()
{
 int moisture = analogRead(VALUE);
 Serial.print("Moisture=");
 Serial.println(moisture);
  if(moisture>MOISTURE_LEVEL1 && moisture<MOISTURE_LEVEL2 )

    {
      analogWrite(10,0);
      analogWrite(11,0);
      analogWrite(9,200);
      digitalWrite(2,LOW);
    }

  else

    {
      if(moisture>MOISTURE_LEVEL2)

      {
        digitalWrite(2,HIGH);
        analogWrite(10,200);
        analogWrite(11,0);
        analogWrite(9,0);
      }

  else

    {
      digitalWrite(2,LOW);
      analogWrite(11,200);
      analogWrite(10,0);
      analogWrite(9,0);
      digitalWrite(3,HIGH);
      delay(200);
      digitalWrite(3,LOW);
      delay(200);
    }

 }

 delay(500); 

 }
