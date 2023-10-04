void setup() {

  pinMode(13, OUTPUT);
  Serial.begin(9600);

}

void loop() {

  for(int i=0;i<255;i++){
    analogWrite(13,i);
    Serial.println(i);
  }  
  for(int i=255;i>0;i--){
    analogWrite(13,i);
    Serial.println(i);
  }

}