
#include <Servo.h>
#define GatePin A3
#define GateLED 9
#define GateLEDPwr 255
#define GateMin 600
#define GateMax 800
#define PotPin A1
#define SwitchPin A2
#define ESCPin 11
#define ServoPin 10
#define CWMaxSpeed 78 
#define CCWMaxSpeed 112
#define Stop 95
#define Spool  10
Servo ESC;
Servo servo;

//ccw vs cw.  6000 = 7.7k DCR, CW. 4.6 CCW 6000 vs 8000 makes same coil size

// ----------------------- Servo Parameters----------------------
int wind_count = 8000;
float arm_length = 13;//arm length from servo center in mm.  19 standard
int underwind = 0; //% underwind  set overwind to 0
int overwind = 0; //% overwind, set underwind to 0
int spin_dir = 1; // set to -1 for Counter Clockwise, and 1 for Clockwise.
float faceplateOffset = 0; // distance off of the faceplate to the inside of the bobbin, good for thicker flatwork.
float error_margin = 5; // Percent of bobbin width to remove for accident prevention.

//-------------------------- Bobbin Parameters-----------------------
float bobbin_width = 10; //Width in Millimeters gap between two faces for winds.
float bobbin_length = 57; // Length of bobbin in mm. just for estimating wire usage.
float bobbin_height = 5; // Width across bobbin in mm. just for esitmating wire usage,

//------------------- Extra parameters---------------

int scatterwind = 4; // Set to 1 for random per Layer, 2 for Pattern per step,.
bool HandWind = false; // Shuts down the servo, for hand winding.  recode for manual servo sweep maybe?

int servoZeroInt = 35; // used to set the minimum point for the servo arm, closest to the plate.
float wire_gauge = .0030; //Wire gauge 0.0030 for heavy  0.0025 standard
float wire_unit = 25.4;  //(set to 25.4 for inches, 1 for mm)
int ScatterMin = 1; // scatterwind steps. for the randomizer,
int ScatterMax = 10;
int ScatterPattern [] = {1,3,7,3,2,1,5,2,1,2};
int Patternlength = 10;
int patternplace = 1;
//---------- Basic setup parameters. Adjustment not required--------
int dir = 1;
int steps = 1;
float deg = 180/3.14;
int rev_mult = 1;
int GateRead = 0;
float Speed = 500;
int count = 0;
int Gateflag = 1;
int PotVal = 0;
int PotValPast = 0;
float RPMpast = 0;
float RPM = 0;
float Dif = 0;
bool Go = true;
int motor_steps = 1;
float DCR_42 = 5.71; // Ohms/m
//  ----------- Startup Calculations -------------------------------
int decrement = 0;
float calculated_width = bobbin_width*(1-error_margin/100);
float sweep = deg*(2*asin(calculated_width/(2*arm_length)));
float servoZero = servoZeroInt+deg*(2*asin(faceplateOffset/(2*arm_length)));
float angle = servoZero + sweep;
float current_angle=servoZero;
int winds = wind_count*(1-underwind/100+overwind/100);
int total_steps = winds*motor_steps;
float wire_dia = wire_gauge*wire_unit;
int layer_winds = calculated_width/wire_dia;
float angle_seg = sweep/layer_winds;
int rev_count = 0;
int pos = 0;
int sysrevs = 1;
float DCR = DCR_42;
//----------------------- Functions ---------------------------------------
void Status(){
Serial.print("Count: ");
Serial.print(rev_count);
Serial.print("   Speed: ");
Serial.print(Speed);

Serial.print("   Current Angle: ");
Serial.print(current_angle);

Serial.print("   Dir: ");
Serial.print(dir);
Serial.print("   Steps: ");   
Serial.print(steps);
Serial.print("   RPM: ");
Serial.println(RPM);
}

void StatusV2(){

if(sysrevs <= rev_count){
  sysrevs = rev_count + 5;
  Status();
}
}


void RangeTest(){
bool active = digitalRead(SwitchPin);
if(active == false){
  PotVal = analogRead(PotPin);

  float range = map(PotVal,0,1023,servoZero,angle);
    if(PotValPast!=range){
    //Serial.print(range);
    //Serial.print("--");
  }
  PotValPast=range;
  servo.write(range);
}}

void ServoControl(){
    current_angle = ServoDir(current_angle,dir,steps);
  servo.write(current_angle);
if(scatterwind == 1){
   steps = random(ScatterMin,ScatterMax);}
if(scatterwind == 3){
  steps = ScatterPattern[patternplace - 1];
  patternplace +=1;
  if(patternplace >= Patternlength){
    patternplace = 1;
  }
}
}
void GateCount(){
GateRead = analogRead(GatePin);
//Serial.println(GateRead);
if(GateRead >= GateMax && Gateflag == 0){
  rev_count = rev_count + 1;
  Gateflag = 1;
  Dif = (millis()-RPMpast)/(1000);
  RPM = 60/(Dif);
  RPMpast = millis();
  //Status();
  ServoControl();

  }

if(GateRead <= GateMin && Gateflag == 1){
Gateflag = 0;  
}}

void SpeedControl(){
  PotVal = analogRead(PotPin);
if(spin_dir == 1 && Go == true){
  Speed = map(PotVal,0,1023,Stop,CWMaxSpeed);}
if(spin_dir == -1 && Go == true){
  Speed = map(PotVal,0,1023,Stop,CCWMaxSpeed);}  
if(Go == false){
    Speed = Stop;
  }
  ESC.write(Speed);

}


int Flip(float in_angle){
        if(in_angle >= angle){
        dir = dir * -1;
        if(scatterwind == 2){
          steps = random(ScatterMin,ScatterMax);}
        if(scatterwind == 4){
  steps = ScatterPattern[patternplace - 1];
  patternplace +=1;
  if(patternplace >= Patternlength){
    patternplace = 1;
  }
}
          
}
      if(in_angle <= servoZero){
        dir = dir * -1;
       }
        return dir;

}

float ServoDir(float current_angle, int dir, int steps){
  float angle_div = steps*angle_seg*rev_mult;
  float out_angle = current_angle + angle_div * dir;

      if(out_angle >= angle){
        out_angle = angle;
        dir = dir * -1;}
      if(out_angle <= servoZero){
        out_angle = servoZero;
        dir = dir * -1;
      }
      
  return out_angle;
}

void Calculate_len(){
  int layers = winds / layer_winds;
  float wire_len = 0;
  
  for (int x = 0; x <= layers ; x+=1){
    float height = wire_dia*x;
    float wind_len = (((bobbin_length-bobbin_height)+height)*2+((bobbin_height+height)*3.14));
    float one_layer = layer_winds*wind_len;
    wire_len+=one_layer;
  }
  float full_len=wire_len;
  float DC = DCR*full_len/1000;
  Serial.print("Winds per layer:  ");
  Serial.println(layer_winds);
  Serial.print("Layers of winds:  ");
  Serial.println(layers);
  Serial.print("Meters of wire:  ");
  Serial.println(full_len);
  Serial.print("Estimated DCR:  ");
  Serial.println(DC);
}


// --------------------Startup code--------------------
void setup()
{  
   Serial.begin(9600);           // set up Serial library at 9600 bps
   Serial.println("------ initializing ------");
   Serial.print("Desired Winds:  ");
   Serial.println(wind_count);
   Serial.print("Bobbin Width in mm:  ");
   Serial.println(bobbin_width);
      Serial.print("Wind Width in mm:  ");
   Serial.println(calculated_width);
   Serial.print("Servo Angle: ");
   Serial.println(sweep);  
      Serial.print("Servo step: ");
   Serial.println(angle_seg);
//      Serial.print("Layer Winds: ");
//   Serial.println(layer_winds);
      Serial.print("Wire Diameter mm: ");
   Serial.println(wire_dia);
   Serial.print("Left Zero: ");
   Serial.print(servoZero);
   Serial.print("   Right limit Zero: ");
   Serial.println(angle);
      Serial.println("----- Estimated Parameters-----");
   Calculate_len();
   
 analogWrite(GateLED,GateLEDPwr);
 ESC.attach(ESCPin);
 servo.attach(ServoPin);     
}

//--------------------- Main code -------------------------------
void loop(){
GateCount();
SpeedControl();
RangeTest();
StatusV2();

if (rev_count < winds){  
if(rev_count < winds-Spool){
  Go = digitalRead(SwitchPin);}

if(Go == true && HandWind == false){
//current_angle = ServoDir(current_angle,dir,steps);
dir = Flip(current_angle);  

          
}}

if(rev_count >= winds-Spool){
Go = false;        
}
if(rev_count >= winds){
    Serial.println("Winding Finished");
    Serial.println(rev_count);
    Calculate_len();
    delay(1000);
    exit(0);}
    }
