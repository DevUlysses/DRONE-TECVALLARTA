char Data[20];
char Char; 
byte index = 0;

void setup(){
  Serial.begin(115200);
  Serial.println("Inicie el Serial");
  delay(1000);
}

void loop(){
  String valor=recibirDATOS();
  if(valor != " "){
    Serial.println(valor);
  }
  delay(100);

}

String recibirDATOS(){

  index=0;

  if(Serial.available() > 0){
    while( Serial.available() > 0 ) {   
      if(index < 19) 
      {
        Char = Serial.read(); // Read a character
        Data[index] = Char; // Store it
        index++; // Increment where to write next
        Data[index] = '\0'; // Null terminate the string
      }
    }
    return Data;
  }
  else{
    return " ";
  }
}




