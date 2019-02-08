int targetStepX = 0;
int curWaitX = 0;
int curStepX = 0;
unsigned int sleepControlX = 0;
bool doneX = 0;
int curSequenceX = 0;
//___________________________________
int targetStepY = 0;
int curWaitY = 0;
int curStepY = 0;
unsigned int sleepControlY = 0;
bool doneY = 0;
int curSequenceY = 0;
//___________________________________
int targetStepZ = 250;
int curWaitZ = 0;
int curStepZ = 0;
unsigned int sleepControlZ = 0;
bool doneZ = 0;
int curSequenceZ = 0;
//___________________________________
bool xOrY = 0;
bool direktion = 0;
const byte numBits = 13;
byte receivedByte[numBits];
bool newData = false;
int result = 0;
bool servoState = 0;
bool servoActive = 0;
//___________________________________
#define SENSOR 3  
int val = 0;              //Variabel til input
int counter = 0;          //Midlertidig variabel der holder styr på antallet af klap
int runder = 0;
bool loopCompleate = 0;
bool service = 0;
//___________________________________
#define yCoil1 8
#define yCoil2 9
#define yCoil3 10
#define yCoil4 11
//___________________________________
#define zCoil1 A1
#define zCoil2 A2
#define zCoil3 A3
#define zCoil4 A4
//___________________________________
#define signalToServo1 A5
#define signalToServo2 2

int stepSequence[2][4]{
     {B00001010, B00001001, B00000101, B00000110},     //Initialiser array med stepretning mod uret
     {B00001010, B00000110, B00000101, B00001001}    //Initialiser array med stepretning med uret
    };

void stepperY(){
  if(direktion == true){
    switch(curSequenceY){
      case 0:
        digitalWrite (yCoil1, HIGH);
        digitalWrite (yCoil2, LOW);
        digitalWrite (yCoil3, HIGH);
        digitalWrite (yCoil4, LOW);
        break;
      case 1:
        digitalWrite (yCoil1, LOW);
        digitalWrite (yCoil2, HIGH);
        digitalWrite (yCoil3, HIGH);
        digitalWrite (yCoil4, LOW);
        break;
      case 2:
        digitalWrite (yCoil1, LOW);
        digitalWrite (yCoil2, HIGH);
        digitalWrite (yCoil3, LOW);
        digitalWrite (yCoil4, HIGH);
        break;
      default:
        digitalWrite (yCoil1, HIGH);
        digitalWrite (yCoil2, LOW);
        digitalWrite (yCoil3, LOW);
        digitalWrite (yCoil4, HIGH);
        break;
    }
  }
  else{
   switch(curSequenceY){ 
    case 0:
        digitalWrite (yCoil1, HIGH);
        digitalWrite (yCoil2, LOW);
        digitalWrite (yCoil3, HIGH);
        digitalWrite (yCoil4, LOW);
        break;
      case 1:
        digitalWrite (yCoil1, HIGH);
        digitalWrite (yCoil2, LOW);
        digitalWrite (yCoil3, LOW);
        digitalWrite (yCoil4, HIGH);
        break;
      case 2:
        digitalWrite (yCoil1, LOW);
        digitalWrite (yCoil2, HIGH);
        digitalWrite (yCoil3, LOW);
        digitalWrite (yCoil4, HIGH);
        break;
      default:
        digitalWrite (yCoil1, LOW);
        digitalWrite (yCoil2, HIGH);
        digitalWrite (yCoil3, HIGH);
        digitalWrite (yCoil4, LOW);
        break;
      }
   }
}

void stepperZ(){
  if(direktion == true){
    switch(curSequenceZ){
      case 0:
        analogWrite (zCoil1, 255);
        analogWrite (zCoil2, 0);
        analogWrite (zCoil3, 255);
        analogWrite (zCoil4, 0);
        break;
      case 1:
        analogWrite (zCoil1, 0);
        analogWrite (zCoil2, 255);
        analogWrite (zCoil3, 255);
        analogWrite (zCoil4, 0);
        break;
      case 2:
        analogWrite (zCoil1, 0);
        analogWrite (zCoil2, 255);
        analogWrite (zCoil3, 0);
        analogWrite (zCoil4, 255);
        break;
      default:
        analogWrite (zCoil1, 255);
        analogWrite (zCoil2, 0);
        analogWrite (zCoil3, 0);
        analogWrite (zCoil4, 255);
        break;
    }
  }
  else{
   switch(curSequenceZ){ 
    case 0:
        analogWrite (zCoil1, 255);
        analogWrite (zCoil2, 0);
        analogWrite (zCoil3, 255);
        analogWrite (zCoil4, 0);
        break;
      case 1:
        analogWrite (zCoil1, 255);
        analogWrite (zCoil2, 0);
        analogWrite (zCoil3, 0);
        analogWrite (zCoil4, 255);
        break;
      case 2:
        analogWrite (zCoil1, 0);
        analogWrite (zCoil2, 255);
        analogWrite (zCoil3, 0);
        analogWrite (zCoil4, 255);
        break;
      default:
        analogWrite (zCoil1, 0);
        analogWrite (zCoil2, 255);
        analogWrite (zCoil3, 255);
        analogWrite (zCoil4, 0);
        break;
      }
   }
}

int acc(int curStepA) {     // Funktion til acceleration
 float fwait = (sqrt((2*(float)curStepA*0.00038)/0.1125)-sqrt((2*((float)curStepA-1)*0.00038)/0.1125)-0.0044)/0.00011;     //Accelerationsformel
 int wait = (int)fwait;   //Omdanner en float til en int
 if (wait < 0) {
  wait = 0;               //Forhindre variablen fra at komme under 0
 }
 return wait;      //Returnere varablen
}

int deAcc(int curStepA) {       //Deaccelerationsfunktion
 float fwait = (sqrt((2*(88-(float)curStepA)*0.00038)/0.1125)-sqrt((2*((88-(float)curStepA)-1)*0.00038)/0.1125)-0.0044)/0.00011;   //Deaccelerationsformel
 int wait = (int)fwait;     //
 if (wait < 0) {
  wait = 0;               //Forhindre variablen fra at komme under 0
 }
 return wait;      //Returnere varablen
}

void recvWithStartEndMarkers() {
    static bool recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    byte rb;
 
    while (Serial.available() > 0 && newData == false) {
        rb = Serial.read();
        if (recvInProgress == true) {
            if (rb != endMarker) {
                receivedByte[ndx] = rb;
                ndx++;
                if (ndx >= numBits) {
                    ndx = numBits - 1;
                }
            }
            else {
              Serial.println((char*)receivedByte);
                receivedByte[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
                int j = 0;
                for (int i = 10; i >= 0; i--){
                  int temp = j;
                  int powerOf2 = 1;
                  for (powerOf2 = 1; temp > 0; temp--){
                    powerOf2 = powerOf2*2;
                    
                  }
                  result=result + (receivedByte[i]-'0')*powerOf2;
                  Serial.println (receivedByte[i]-'0');
                  Serial.println (powerOf2);
                  Serial.println (result);
                  j ++;
                }
            }
        }

        else if (rb == startMarker) {
            recvInProgress = true;
        }
    }
}

void showNewData() {
    if (newData == true && xOrY == 0) {
      targetStepX = result;
      curStepX = targetStepX; 
      newData = false; 
      xOrY = 1;
      result = 0;
    }
    else if(newData == true && xOrY == 1){
      targetStepY = result;
      curStepY = targetStepY;
      newData = false;
      xOrY = 0;
      result = 0;
    }
}

void servoOpen(){
  servoActive = 1;
  Serial.write ("Jeg er her");
  for( int i = 2; i> 0; i--){ 
  analogWrite (signalToServo1, 255);
  digitalWrite (signalToServo2, HIGH);
  delay (1);
  digitalWrite (signalToServo2, LOW);
  delay (1);
  analogWrite (signalToServo1, 0);
  delay(18);
  }
  servoState = true;
  targetStepX = 0;
  targetStepY = 0;
  doneX = 0;
  doneY = 0;
  doneZ = 0;
  loopCompleate = 0;
  servoActive = 0;
}
void servoClose(){
  servoActive = true;
  Serial.write ("Jeg er her");
  for( int i = 2; i> 0; i--){ 
  analogWrite (signalToServo1, 255);
  digitalWrite (signalToServo2, HIGH);
  delay (1);
  analogWrite (signalToServo1, 0);
  delay(1);
  digitalWrite (signalToServo2, LOW);
  delay(18);
  }
  servoState = false;
  servoActive = 0;
}


void setup() {                  //Initialisere output i Arduinoens bus der styrer pin 0 til 7
  DDRD = DDRD | B11110000;      //her sættes pin 7 til 4 som output uden at røre ved de fire første pins der styrer Arduinoens kommunikation
  DDRB = DDRB | B001111;
  Serial.begin(115220);
  Serial.write("Ardunio startet");
  
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1<<CS11);
  TIMSK1 |= (1<<TOIE1);
  TCNT1 = 45536;

  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2B |= (1<<CS22);
  TCCR2B |= (1<<CS21);
  TIMSK2 |= (1<<TOIE2);
  TCNT2 = 187;
  servoOpen();
  servoClose();
  servoOpen();
}

void loop() {           //Selve programmet der kalder funktionerne
  while(service == 0){
   val = digitalRead(SENSOR);   //læser sensorens data
   runder ++;                   //inkrementer tiden siden sidste klap
   if(val ==0){            //Hvis der er en høj lyd inkrementere vi antallet af klap og resetter tidsvariablen
    Serial.println (val);
    counter ++;
    runder = 0; 
    }   
   if (counter ==3){       //Hvis der er blevet lavet tre klap resetter vi klappene og går ud af loopet 
    counter = 0;
    service = 1;
      Serial.write ("k");
    }
   if (runder == 100){     // Hvis der er gået 100 runder eller ca. 3 sekunder siden sidste klap, reset tidsvariablen og antallet af klap
    runder = 0;
    counter = 0;
    }
   delay (30);                //Gentager loopet efter 30 millisekunder
   
  }
  val = 0;
  counter = 0;
  runder = 0;
  while (service == 1 && loopCompleate == 0){
    recvWithStartEndMarkers();
    showNewData();
  }
    service = 0;
}


ISR(TIMER1_OVF_vect){
  
  TCNT1 = 45536; 
}

ISR(TIMER2_OVF_vect){
  if (servoActive == false){
  direktionX();
  direktionY();
  if (doneZ == true && servoState == true){
  servoClose();
  direktion = true;
  doneZ = false;
  curStepZ = targetStepZ;
    Serial.print("servo lukket");
  }    
  else if (doneX == true && doneY == true && doneZ == false){
    Serial.write ("Jeg er her");
    direktionZ();
  }
  
  else if (doneX == true && doneY == true && servoState == false){
      servoOpen();
      analogWrite(zCoil1, 0);
      analogWrite(zCoil2, 0);
      analogWrite(zCoil3, 0);
      analogWrite(zCoil4, 0);
    }
  }
  TCNT2  = 187; 
}

void direktionX(){     //Funktion til bevægelse i x-retningen
  if (curStepX > 0) {
    PORTD = PORTD%B00010000 + ((stepSequence[direktion][curSequenceX]) << 4);    //Hvis vi ikke er ved den ønskede position laves et step
    if (curWaitX <= 0) {           //Tjekker om motoren er klar til et step, bruges under acceleration
      if (curSequenceX < 3) {      //Sørger for at motoren tager et helt step
        curSequenceX++;
      } else {                    //Når motoren har taget et step resettes den
        curSequenceX = 0;
        if(targetStepX-curStepX <= targetStepX/2){   //Hvis kloen har bevæget sig under halvvejs til målet bruges accelerationsfunktionen 
          curWaitX = acc(targetStepX-curStepX);      //Accelerationsfunktionen kaldes for at bestemme tiden til næste step
          }
          else{                                   
           if (curStepX > 88){                     //Hvis kloen er over halvvej til målet og er over 88 steps fra målet bruges accelerationsfunktionen
            curWaitX = acc(targetStepX-curStepX);
            }
           else{                                  //Hvis kloen er over halvvejs men under 88 steps fra målet bruges deaccelerationsfunktionen
            curWaitX = deAcc(88-curStepX);          //Deccelerationsfunktionen kaldes for at bestemme tiden til næste step
            }  
          }
          curStepX--;                //Dekrementere antallet af steps tilbage
        }
      } else {                      //Hvis motoren ikke er klar til et nyt step slukkes motoren og ventetiden decrementeres
        PORTD = PORTD%B00010000 + (B00000000 << 4);
        curWaitX--;
      }
      if (curStepX == 0 && targetStepX > 0){
        doneX = 1;
        curStepZ = targetStepZ;
      }
      else {
        doneX = 0;
      }
      sleepControlX = 0;             //Resetter en variabel der holder styr på om motoren har bevæget sig siden sidst
  } else {              
     if ((float)sleepControlX > 10000/1.1) {   //Hvis kloen er ved destinationen i x-retningen tjekkes det om der er gået 10 sekunder
        PORTD = PORTD%B00010000 + (B00000000 << 4);   //Hvis der er gået 10 sekunder slukkes motoren
      } else if (sleepControlX%9 == 0) {               //Hvis der ikke er gået 10 skunder tændes motoren som bremse hver 9. cyklus og tiden inkrementeres
        PORTD = PORTD%B00010000 + ((stepSequence[direktion][curSequenceX])<< 4);
        sleepControlX++;
      } else {                                        //Hvis der ikke er gået 10 sekunder og det ikke er den 9. cyklus slukkes motoren og tiden inkrementeres
        PORTD = PORTD%B00010000 + (B00000000 << 4);
        sleepControlX++;
      }
    }
} 

void direktionY(){     //Funktion til bevægelse i x-retningen
  if (curStepY > 0) {
    stepperY();    //Hvis vi ikke er ved den ønskede position laves et step
    if (curWaitY <= 0) {           //Tjekker om motoren er klar til et step, bruges under acceleration
      if (curSequenceY < 3) {      //Sørger for at motoren tager et helt step
        curSequenceY++;
      } else {                    //Når motoren har taget et step resettes den
        curSequenceY = 0;
        if(targetStepY-curStepY <= targetStepY/2){   //Hvis kloen har bevæget sig under halvvejs til målet bruges accelerationsfunktionen 
          curWaitY = acc(targetStepY-curStepY);      //Accelerationsfunktionen kaldes for at bestemme tiden til næste step
          }
          else{                                   
           if (curStepY > 88){                     //Hvis kloen er over halvvej til målet og er over 88 steps fra målet bruges accelerationsfunktionen
            curWaitY = acc(targetStepY-curStepY);
            }
           else{                                  //Hvis kloen er over halvvejs men under 88 steps fra målet bruges deaccelerationsfunktionen
            curWaitY = deAcc(88-curStepY);          //Deccelerationsfunktionen kaldes for at bestemme tiden til næste step
            }  
          }
          curStepY--;                //Dekrementere antallet af steps tilbage
        }
      } else {                      //Hvis motoren ikke er klar til et nyt step slukkes motoren og ventetiden decrementeres
        PORTB = ((PORTB >> 4) << 4) + B00000000;
        curWaitY--;
      }
      if (curStepY == 0 && targetStepY > 0){
        doneY = 1;
      }
      else {
        doneY = 0;
      }
      sleepControlY = 0;             //Resetter en variabel der holder styr på om motoren har bevæget sig siden sidst
  } else { 
                 
    if ((float)sleepControlY > 10000/4) {   //Hvis kloen er ved destinationen i x-retningen tjekkes det om der er gået 10 sekunder
        PORTB = ((PORTB >> 4) << 4) + B00000000;   //Hvis der er gået 10 sekunder slukkes motoren
      } else if (sleepControlY%9 == 0) {               //Hvis der ikke er gået 10 skunder tændes motoren som bremse hver 9. cyklus og tiden inkrementeres
        PORTB = ((PORTB >> 4) << 4) + stepSequence[direktion][curSequenceY];
        sleepControlY++;
      } else {                                        //Hvis der ikke er gået 10 sekunder og det ikke er den 9. cyklus slukkes motoren og tiden inkrementeres
        PORTB = ((PORTB >> 4) << 4) + (B00000000 >> 4);
        sleepControlY++;
      }
    }
} 


void direktionZ(){
    if (curStepZ > 0) {
    stepperZ();    //Hvis vi ikke er ved den ønskede position laves et step
    if (curWaitZ <= 0) {           //Tjekker om motoren er klar til et step, bruges under acceleration
      if (curSequenceZ < 3) {      //Sørger for at motoren tager et helt step
        curSequenceZ++;
      } else {                    //Når motoren har taget et step resettes den
        curSequenceZ = 0;
        if(targetStepZ-curStepZ <= targetStepZ/2){   //Hvis kloen har bevæget sig under halvvejs til målet bruges accelerationsfunktionen 
          curWaitZ = acc(targetStepZ-curStepZ);      //Accelerationsfunktionen kaldes for at bestemme tiden til næste step
          }
          else{                                   
           if (curStepZ > 88){                     //Hvis kloen er over halvvej til målet og er over 88 steps fra målet bruges accelerationsfunktionen
            curWaitZ = acc(targetStepZ-curStepZ);
            }
           else{                                  //Hvis kloen er over halvvejs men under 88 steps fra målet bruges deaccelerationsfunktionen
            curWaitZ = deAcc(88-curStepZ);          //Deccelerationsfunktionen kaldes for at bestemme tiden til næste step
            }  
          }
          curStepZ--;                //Dekrementere antallet af steps tilbage
        }
      } else {                      //Hvis motoren ikke er klar til et nyt step slukkes motoren og ventetiden decrementeres
        digitalWrite (zCoil1, LOW);
        digitalWrite (zCoil2, LOW);
        digitalWrite (zCoil3, LOW);
        digitalWrite (zCoil4, LOW);
        curWaitZ--;
      }
      if (curStepZ == 0 && targetStepZ > 0){
        doneZ = 1;
      }
      else {
        doneZ = 0;
      }
      if (doneZ == 1 && servoState == false){
        doneX = 0;
        doneY = 0;
        curStepX = targetStepX;
        curStepY = targetStepY;
      }
      sleepControlZ = 0;             //Resetter en variabel der holder styr på om motoren har bevæget sig siden sidst
  } else { 
                 
    if ((float)sleepControlZ > 10000/4) {   //Hvis kloen er ved destinationen i x-retningen tjekkes det om der er gået 10 sekunder
        digitalWrite (zCoil1, LOW);
        digitalWrite (zCoil2, LOW);
        digitalWrite (zCoil3, LOW);
        digitalWrite (zCoil4, LOW);   //Hvis der er gået 10 sekunder slukkes motoren
      } else if (sleepControlZ%9 == 0) {               //Hvis der ikke er gået 10 skunder tændes motoren som bremse hver 9. cyklus og tiden inkrementeres
        digitalWrite (zCoil1, LOW);
        digitalWrite (zCoil2, HIGH);
        digitalWrite (zCoil3, LOW);
        digitalWrite (zCoil4, HIGH);
        sleepControlZ++;
      } else {
        digitalWrite (zCoil1, LOW);
        digitalWrite (zCoil2, LOW);
        digitalWrite (zCoil3, LOW);
        digitalWrite (zCoil4, LOW);//Hvis der ikke er gået 10 sekunder og det ikke er den 9. cyklus slukkes motoren og tiden inkrementeres
        sleepControlZ++;
      }
    }
}




