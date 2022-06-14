/*
   bluetooth + afstand
*/

// pinnen voor de afstand sensoren
int Lecho1 = 40; //links echo
int Ltrig1 = 41; //links trig
int Fecho2 = 43; //forward echo
int Ftrig2 = 42; //forward trig
int Recho3 = 44; //rechts echo
int Rtrig3 = 45; //rechts trig
int Becho4 = 38; // niet meer nodig
int Btrig4 = 39; // niet meer nodig


int L_Distance = 0; // in deze varaible wordt de waarde van de afstand opgeslaan
int F_Distance = 0; // in deze varaible wordt de waarde van de afstand opgeslaan
int R_Distance = 0; // in deze varaible wordt de waarde van de afstand opgeslaan
int B_Distance = 0; // in deze varaible wordt de waarde van de afstand opgeslaan

#include <SoftwareSerial.h>
SoftwareSerial BTserial(19, 18); // RX | TX

/*
   vuur sensoor
*/

#include <Wire.h>
#include <Adafruit_MLX90614.h>

// adres van een van de temperatuur sensoren
// dezelfde pin maar hebben verschillende adressen
#define IR1 0x5B  
#define IR2 0x55
#define IR3 0x5A
// arduino functie
Adafruit_MLX90614 mlx; 

/*
   motor + ps2
*/

#define enA 53
#define in1 47
#define in2 49
#define in3 35
#define in4 37
#define enB 51
int LX = 0;
int LY = 0;
int RX = 0;
int RY = 0;
int len = 0;

#include <PS2X_lib.h>

int count = 1;
int error = 0;
byte type = 0;
byte vibrate = 0;
int check = 0;
PS2X ps2x;

/*
   stepper motor
*/

#include <Stepper.h>
const int stepsPerRevolution = 200; // change this to fit the number of steps per revolution
Stepper myStepper(stepsPerRevolution, 22, 23, 24, 25); //initialize the stepper library on pins 22 through 25:
const int enAStepper = 46;
const int enBStepper = 46;

/*
   water pomp
*/

#include <AFMotor.h>
AF_DCMotor pump(2);

void setup() {

  /*
     stepper motor
  */

  analogWrite(enAStepper, 255);
  analogWrite(enBStepper, 255);
  myStepper.setSpeed(60);

  /*
     ps2
  */

  Serial.begin(9600);
  //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(31, 32, 33, 34, true, true); 
  type = ps2x.readType();

  if (error == 0) {
    Serial.println("Found Controller, configured successful");
  }
  else if (error == 1)
    Serial.println("No controller found, check wiring or reset the Arduino");
  else if (error == 2)
    Serial.println("Controller found but not accepting commands");
  else if (error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it.");

  // Check for type of controller
  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.println("Unknown Controller type");
      break;
    case 1:
      Serial.println("DualShock Controller Found");
      break;
    case 2:
      Serial.println("GuitarHero Controller Found");
      break;
  }

  /*
     afstand
  */

  //obstacle sensor
  pinMode(Ltrig1, OUTPUT);
  pinMode(Lecho1, INPUT);
  pinMode(Ftrig2, OUTPUT);
  pinMode(Fecho2, INPUT);
  pinMode(Rtrig3, OUTPUT);
  pinMode(Recho3, INPUT);

  BTserial.begin(9600);
  pinMode(Btrig4, OUTPUT);
  pinMode(Becho4, INPUT);

  /*
     motoren
  */

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  /*
     vuur
  */

  mlx.begin(); // functie beginnen
}

/*
   Methode voor afstandsensoren
*/

// methode voor de linkse afstand te berekenen

int L_Distance_test() {
  digitalWrite(Ltrig1, LOW);
  delayMicroseconds(2);
  digitalWrite(Ltrig1, HIGH);
  delayMicroseconds(10);
  digitalWrite(Ltrig1, LOW);
  float Ldistance = pulseIn(Lecho1, HIGH);
  delay(10);

  Ldistance = Ldistance  * 0.034 / 2;
  return (int)Ldistance;
}

// methode voor de forward afstand te berekenen

int F_Distance_test() {
  digitalWrite(Ftrig2, LOW);
  delayMicroseconds(2);
  digitalWrite(Ftrig2, HIGH);
  delayMicroseconds(10);
  digitalWrite(Ftrig2, LOW);
  float Fdistance = pulseIn(Fecho2, HIGH);
  delay(10);

  Fdistance = Fdistance  * 0.034 / 2;
  return (int)Fdistance;
}

// methode voor de rechtse afstand te berekenen

int B_Distance_test() {
  digitalWrite(Btrig4, LOW);
  delayMicroseconds(2);
  digitalWrite(Btrig4, HIGH);
  delayMicroseconds(10);
  digitalWrite(Btrig4, LOW);
  float Bdistance = pulseIn(Becho4, HIGH);
  delay(10);

  Bdistance = Bdistance  * 0.034 / 2;
  return (int)Bdistance;
}

// methode voor de rechtse afstand te berekenen

int R_Distance_test() {
  digitalWrite(Rtrig3, LOW);
  delayMicroseconds(2);
  digitalWrite(Rtrig3, HIGH);
  delayMicroseconds(10);
  digitalWrite(Rtrig3, LOW);
  float Rdistance = pulseIn(Recho3, HIGH);
  delay(10);

  Rdistance = Rdistance  * 0.034 / 2;
  return (int)Rdistance;
}

/*
   methode voor motoren
*/

void forward()
{
  analogWrite(enA, 200);
  analogWrite(enB, 200);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void backward()
{
  analogWrite(enA, 200);
  analogWrite(enB, 200);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void right()
{
  analogWrite(enA, 255);
  analogWrite(enB, 80);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void left()
{
  analogWrite(enA, 80);
  analogWrite(enB, 255);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void stopm() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

}

/*
   Methode stepper motor
*/

void setStepperIdle() {
  digitalWrite(50, LOW);
  digitalWrite(51, LOW);
  digitalWrite(52, LOW);
  digitalWrite(53, LOW);
}

void StepperPwmOff() {
  analogWrite(enAStepper, 0);
  analogWrite(enBStepper, 0);
}

void StepperPwmOn() {
  analogWrite(enAStepper, 255);
  analogWrite(enBStepper, 255);
}

void pumpstart() {
  pump.setSpeed(255);
  pump.run(FORWARD);
}
void pumpstop() {
  pump.setSpeed(0);
  pump.run(RELEASE);
}

void loop()
{
  if (error == 1) {
    //skip loop if no controller found
  }
  else {
    ps2x.read_gamepad(false, vibrate);
    delay(50);

    //receive values from p22 joystick
    LY = ps2x.Analog(PSS_LY);                     
    LX = ps2x.Analog(PSS_LX);

    RY = ps2x.Analog(PSS_RY);
    RX = ps2x.Analog(PSS_RX);

    stopm();
    setStepperIdle();

    //check if the joystick pushed up side
    if (LY > 200)                      
    {
      forward();
      Serial.println("Back");
    }

    if ((LY < 100 ) && (LX != 0))
    {
      backward();
      Serial.println("for");
      Serial.println(LY);
    }

    if ((LX < 100 ) && (LY != 0))
    {
      right();
      Serial.println("left");
      Serial.println(LX);
    }
    if (LX > 200 )
    {
      left();
      Serial.println("right");
    }
    if (RX > 200) {
      StepperPwmOn();
      myStepper.step(-5);
      StepperPwmOff();
      setStepperIdle();
      Serial.println("turn left");
    }

    if ((RX < 100)  && (LY != 0)) {
      StepperPwmOn();
      myStepper.step(5);
      StepperPwmOff();
      setStepperIdle();
      Serial.println("turn right");
      Serial.println(RX);
    }

    if (ps2x.ButtonPressed(PSB_R1)) {
      Serial.println("pump_start");
      pumpstart();
    }

    if (ps2x.ButtonPressed(PSB_R2)) {
      Serial.println("pump_stop");
      pumpstop();
    }

    if (ps2x.ButtonPressed(PSB_L1))
    {
      Serial.println("press");
      count = 1;
      while (count < 10)
      {
        count++;
        Serial.println(count);

        /*
          bluetooth
        */

        R_Distance = R_Distance_test();//hier wordt de afstand bepaald
        L_Distance = L_Distance_test();//hier wordt de afstand bepaald
        B_Distance = B_Distance_test();//hier wordt de afstand bepaald
        F_Distance = F_Distance_test();//hier wordt de afstand bepaald

 	      /*
          vuur sensor
        */ 

        mlx.AddrSet(IR1); // adres set sensor 1
        mlx.temp1 = mlx.readObjectTempC(); // temperatuur lezen sensor 1

        mlx.AddrSet(IR2); // adres set sensor 2
        mlx.temp2 = mlx.readObjectTempC(); // temperatuur lezen sensor 2

        mlx.AddrSet(IR3); // adres set sensor 3
        mlx.temp3 = mlx.readObjectTempC(); // temperatuur lezen sensor 3
        
        BTserial.print(F_Distance);
        BTserial.print("cm");
        BTserial.print("|");

        BTserial.print(B_Distance);
        BTserial.print("cm");
        BTserial.print("|");

        BTserial.print(L_Distance);
        BTserial.print("cm");
        BTserial.print("|");

        BTserial.print(R_Distance);
        BTserial.print("cm");
        BTserial.print("|");

        BTserial.print(mlx.temp1);
        BTserial.print("*C");
        BTserial.print("|");

        BTserial.print(mlx.temp2);
        BTserial.print("*C");
        BTserial.print("|");

        BTserial.print(mlx.temp3);
        BTserial.print("*C");
        BTserial.print("|");

        delay(400);
      }
      LY = LX = 128; //return to default vlaues
      RY = RX = 128; //return to default values
    }
  }
}
