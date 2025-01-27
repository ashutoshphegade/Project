
void setup() 
{
pinMode(2,OUTPUT);
pinMode(3,OUTPUT);

pinMode(4,OUTPUT);
pinMode(5,OUTPUT);

pinMode(6,OUTPUT);
pinMode(7,OUTPUT);

pinMode(0,OUTPUT);
pinMode(1,OUTPUT);
}


void loop()
{
  int a = analogRead(A0);
  int b = analogRead(A1);
  int c = analogRead(A2);
  int d = analogRead(A3);
  
 if(a<100)
  {
     digitalWrite(0,HIGH);
     digitalWrite(1,LOW);
  }

  else if(a>1000)
  {
     digitalWrite(1,HIGH);
     digitalWrite(0,LOW);
 }

  else if(b<100)
  {
     digitalWrite(2,HIGH);
     digitalWrite(3,LOW);
 }

 else if(b>1000)
  {
     digitalWrite(2,LOW);
     digitalWrite(3,HIGH);
   }

  else if(c<100)
  {
     digitalWrite(4,HIGH);
     digitalWrite(5,LOW);
  } 

  else if(c>1000)
  {
     digitalWrite(5,HIGH);
     digitalWrite(4,LOW);
  }

  else if(d<100)
  {
     digitalWrite(6,HIGH);
     digitalWrite(7,LOW):
  }

 else if(d>1000)
  {
     digitalWrite(6,LOW);
     digitalWrite(7,HIGH);
  }  

  else
  {
     digitalWrite(3,LOW);
     digitalWrite(2,LOW);
     digitalWrite(4,LOW);
     digitalWrite(5,LOW);
     digitalWrite(6,LOW);
     digitalWrite(7,LOW);
     digitalWrite(0,LOW);
     digitalWrite(1,LOW);
  }
}

