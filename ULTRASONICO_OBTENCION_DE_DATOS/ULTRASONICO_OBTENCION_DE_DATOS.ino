const int trigPin = 9;
const int echoPin = 10;

long tiempo;
int salida;

void setup()
{
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  tiempo = pulseIn(echoPin, HIGH);

  if(tiempo < 1000)
  {
    salida = 1;
  }
  else
  {
    salida = 0;
  }

  Serial.print("Entrada sensor (us): ");
  Serial.print(tiempo);

  Serial.print("   Salida: ");
  Serial.println(salida);

  delay(300);
}