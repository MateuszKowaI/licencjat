#define TRIG_PIN 2
#define ECHO_PIN 3

long duration;
int distance;

void setup() {
  Serial.begin(9600); 
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  digitalWrite(TRIG_PIN, LOW);  
  delayMicroseconds(2);  
  digitalWrite(TRIG_PIN, HIGH);  
  delayMicroseconds(10);  
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  
  distance = duration * 0.034 / 2;

  Serial.println(distance);

  delay(50);
}