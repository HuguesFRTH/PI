#include <MechaQMC5883.h>

#include <Servo.h>

// MODE MANUEL
int upBouton = 4;
int downBouton = 5;
int leftBouton = 6;
int rightBouton = 7;

int modeBouton = 8;

int mode = 0;

bool autoC = true;
 

//--------------------GYRO ET BOUSSOLE----------------


MechaQMC5883 qmc;

const int MPU_addr=0x68; int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int minVal=265; int maxVal=402;

double xG; double yG; double zG;

// ------------------------------------

bool inisFinish = false;

float tour = 1385;// temps en milisecondes que met le moteur pour faire 1 tour.

bool hauteurImpo = false;

//Les différents messages qui seront affichés ...
String messages[3] = {
"   J'observe    ",
"   Je tourne    ",
""
};

//ECRAN LCD

#include <Wire.h>
#include<LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F, 20, 4);


int nbrePlaneteTAB = 9;
String planetes[] = {
  "    Neptune    ",//0
  "    Mars       ",//1
  "    Mercure    ",//2
  "    Venus      ",//3
  "    Jupiter    ",//4
  "    Saturne    ",//5
  "    Urnanus    ",//6
  "    Pluton     ",//7
  "    Lune       " //8
};

int currentPlaneteID = 444;
float time_cmd = 0;
Servo Serv_Haut;//servo ^
Servo Serv_Bas; //servo > 

  
float PosX = 0;
float PosY = 0;                                       


String incomingByte="";
int sens = 0;
int sens_sup = 0;
int sens_inf = 180;
float tour_Haut = 1385;// temps en milisecondes que met le moteur pour faire 1 tour.
float tour_Bas = 1385;// temps en milisecondes que met le moteur pour faire 1 tour.
//Hauteur et azimut reçu par USB
float hauteurIncoming, azimutIncoming;
void setup()   {

//MODE MANUEL
pinMode(upBouton,INPUT_PULLUP);
pinMode(downBouton,INPUT_PULLUP);
pinMode(leftBouton,INPUT_PULLUP);
pinMode(rightBouton,INPUT_PULLUP);
pinMode(modeBouton,INPUT_PULLUP);

  
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true); 
  qmc.init();
  qmc.setMode(Mode_Continuous,ODR_200Hz,RNG_2G,OSR_256);

  
  //declaration du servo gérant déclinaison/hauteur/inclinaison  ^                                               
  Serv_Haut.attach(2,1000, 2000);
  pinMode(2, OUTPUT);
  Serv_Haut.write(90);
  //declaration du servo gérant l'azunite/  >
  Serv_Bas.attach(3,1000, 2000);
  pinMode(3, OUTPUT);
  Serv_Bas.write(90);
  //début de la transmission via le baudseet 115200 
  Serial.begin(115200);
   //inis();
  lcd.init();
  updateScreen();
}



void loop()
{
  autoC = digitalRead(modeBouton);
  if(autoC){
    if(!inisFinish){
      inisScreen();
    }
    while(!inisFinish){
      inis();
    }
    //Serial.println("end");
    traitementByte();
  }
  else{
  //  manuel();
  }
}









void traitementByte()
{
  if(Serial.available())
  {
    incomingByte=Serial.readString();
    Serial.print("J'ai reçu ");
    Serial.println(incomingByte);
    //Séparation de l'azimut et hauteur : hauteur/azimut
    int cp = 0;
    int ct = 0;
    for(int i = 0; i < incomingByte.length();i ++){
      
      if(incomingByte.charAt(i) == '/')
      {
        cp = i;
        if(ct == 0)
        {
          currentPlaneteID = (incomingByte.substring(ct-1,0)).toInt();
          Serial.print("Planete ID = ");
          Serial.println(currentPlaneteID);
          ct = i;
        }
        else
        {
          Serial.print("Hauteur = ");
          hauteurIncoming = (incomingByte.substring(cp,ct+1)).toFloat();
          Serial.println(hauteurIncoming);
          Serial.print("Azimut = ");
          azimutIncoming = (incomingByte.substring(i+1)).toFloat();
          Serial.println(azimutIncoming);
        }
      }
}

updateScreen();
String message;
//HAUTEUR
float moveX, moveY;

moveX =  PosX - hauteurIncoming;
if(moveX < 0)
        {
          sens=sens_inf;
          moveX = sqrt(sq(moveX));
        }
        else{ if (moveX > 0)
          {
            sens=sens_sup;
          }}
          if(hauteurIncoming < 0){
            hauteurImpo = true;
          }else{
            hauteurImpo = false;
            
            time_cmd = (moveX*tour_Haut/360) * 100;
            message = "Hauteur tourne de " + String(moveX) + " dans le sens " + String(sens) + " pendant " + String(time_cmd) + " milisecondes";
            Serial.println(message);
            Serv_Haut.write(sens);
            delay(time_cmd);
            Serv_Haut.write(90);
            PosX = PosX + hauteurIncoming;
            
            //AZIMUT
            moveY = PosY - azimutIncoming;
            if(moveY < 0)
                    {
                      sens=sens_sup;
                      moveY = sqrt(sq(moveY));
                    }
                    else{  if (moveY > 0)
                      {
                        sens=sens_inf;
                      }
                      }
            time_cmd = (moveY*tour_Bas/360) * 100;
            
            message = "Azimut tourne de " + String(moveY) + " dans le sens " + String(sens) + " pendant " + String(time_cmd) + " milisecondes";
            Serial.println(message);
            Serv_Bas.write(sens);
            delay(time_cmd);
            Serv_Bas.write(90);
            PosY = PosY + azimutIncoming;
            
          }
updateScreen();
Serial.println("end");
 
}
  else
  {
    
  // Serial.println("Rien");
    Serial.flush();
  }
 } 



 void updateScreen(){
  lcd.backlight();
  // MODE AUTO
  if(autoC){

if(!hauteurImpo){
     lcd.setCursor(0,0);
     lcd.print("   J'observe    ");
  }
  else{
     lcd.setCursor(0,0);
     lcd.print("  Impossible :  ");
  }
     if(nbrePlaneteTAB <= currentPlaneteID) {
      
      lcd.setCursor(0,1);
     lcd.print("      ----      ");
    }
    else{
      lcd.setCursor(0,1);
     lcd.print(planetes[currentPlaneteID]);
    }}
  // MODE MANUEL
  else{
    
  }

 }


 void inisScreen(){
    lcd.backlight();
    lcd.setCursor(0,0);
    lcd.print("*INITIALISATION*");
 }

 void inis(){
  inisFinish = false;
  int x, y, z;
  float azimuth;
  //float azimuth; //is supporting float too
  qmc.read(&x, &y, &z);
  azimuth = atan2((int)x,(int)y) * 180.0/PI;
  Serial.print(" azimuth: ");
  Serial.print(azimuth);
  Serial.println();
  Wire.beginTransmission(MPU_addr); Wire.write(0x3B); Wire.endTransmission(false); Wire.requestFrom(MPU_addr,14,true); AcX=Wire.read()<<8|Wire.read(); AcY=Wire.read()<<8|Wire.read(); AcZ=Wire.read()<<8|Wire.read(); int xAng = map(AcX,minVal,maxVal,-90,90); int yAng = map(AcY,minVal,maxVal,-90,90); int zAng = map(AcZ,minVal,maxVal,-90,90);

xG= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI); yG= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI); zG= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

//Serial.print("AngleX= "); Serial.println(xG);
//
//Serial.print("AngleY= "); Serial.println(yG);
//
//Serial.print("AngleZ= "); Serial.println(zG); Serial.println("-----------------------------------------"); 
if((int)(azimuth) == 90){
  Serv_Bas.write(90);
}else {
  if( azimuth< 90){
     Serv_Bas.write(180);
  }else{
     Serv_Bas.write(0);
  }  
  }
Serial.println((xG));

if((int)(xG) == 90){
  Serv_Haut.write(90);
}else {
  if(xG < 90){
     Serv_Haut.write(0);
  }else{
     Serv_Haut.write(180);
  }
}
if((int)(xG) == 90 && (int)(azimuth) == 90){
  inisFinish = true;
  updateScreen();
}
//modee();
if(!autoC){
  inisFinish = true;
  updateScreen();
}
delay(400); 
}

void manuel(){
bool upSt =   digitalRead(upBouton);
bool downSt =  digitalRead(downBouton);
 bool leftSt = digitalRead(leftBouton);
 bool rightSt = digitalRead(rightBouton);
  if(upSt && !downSt){
    Serv_Haut.write(sens_sup);
  }
  if(!upSt && downSt){
    Serv_Haut.write(sens_inf);
  }
   if(upSt && downSt){
    Serv_Haut.write(90);
  }
  if(!leftSt && rightSt){
    Serv_Bas.write(sens_sup);
  }
  
  if(leftSt && !rightSt){
    Serv_Bas.write(sens_inf);   
  }
  
   if(leftSt && rightSt){
    Serv_Bas.write(90);
  }
}
