const int trigPin = 42;
const int echoPin = 44;
int ultrasonicDelay = 0;
float duration, distance;
int PreviousDistance =0;
int NewDistance = 0;
bool objectFound = false;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(115200);
}

void loop() {
  ultrasonicDelay++;
        if (ultrasonicDelay >= 2000){
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        duration = pulseIn(echoPin, HIGH);
        distance = (duration*.0343)/2;
        Serial.println(distance);
        ultrasonicDelay = 0;

        /*PreviousDistance = NewDistance;
        NewDistance = distance;
        if ((PreviousDistance - NewDistance)>=60){
          objectFound = true;
          while (objectFound == true){
            Serial.println("Object Found");
          }*/      
       // }
    }
}