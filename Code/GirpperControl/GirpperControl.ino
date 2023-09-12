#include <HX710B.h>

// Pins
#define pump_A 8  //in1 pump
#define pump_B 9  //in2 pump
#define valve_1 6 //in3 Intake valve
#define valve_2 7 //in4 Exhaust valve

int Pressure = 0; //initialize reading
int Pressure_t = 9000;  //target pressure
int pwm = 63;  //6V output for pump

//initialization of controllor
int Err;
double errorTolerance = 0.05 * Pressure_t;  //5% torlerance

// initialization of PID
double kp = 1, ki = 0, kd = 0.1;
long sumErr;
int lastErr;
// -----------------------------------------------------------------------------
// Function(s)

double PID(int Err, int sumErr, int lastErr)
{
  
}

void setup() {
  // Pins for pump setup
  pinMode(pump_A, OUTPUT);
  digitalWrite(pump_A, 0);
  pinMode(pump_B, OUTPUT);
  digitalWrite(pump_B, 0);

  // Valve 1 pins setup
  pinMode(valve_1, OUTPUT);
  digitalWrite(valve_1, 0);

  // Valve 2 pins setup
  pinMode(valve_2, OUTPUT);
  digitalWrite(valve_2, 0);

  // System initialization： Releae pressure
  digitalWrite(valve_1,1);
  digitalWrite(valve_2,1);
  delay(1000);
  digitalWrite(valve_1,127);
  digitalWrite(valve_2,0);

  // Pressure sensor initialization
  Serial.begin(9600);
  Init_Hx710();
  Get_Rough();  //clear the pressure
  //delay(3000);
}

void loop() {
  // read pressure from HX710
  Pressure = Get_Pressure();
  Serial.println(Pressure); //print pressure
  delay(5);

  // Calculate error
  Err = Pressure_t - Pressure;
  Serial.print("Err= ");
  Serial.println(Err);

//  analogWrite(pump_A,63);
//  analogWrite(pump_B,0);

  // control pump and valve base on error

if (Err > 20) // Pressure is not enough, 20 is the buffer
{
  digitalWrite(valve_1, 1);  //open the intake valve
  digitalWrite(valve_2, 0);   //close the exhaust valve
  analogWrite(pump_A, 63);   //pump on
  analogWrite(pump_B, 0);
  Serial.println("intaking");
}
else if (Err < -700) //Presure is too high, 20 is the buffer
{
  digitalWrite(valve_1, 0);  //close the intake valve
  digitalWrite(valve_2, 1);   //open the exhaust valve
  analogWrite(pump_A, 63);   //pump off
  analogWrite(pump_B, 0);
  delay(50);
  digitalWrite(valve_2, 0);   //close the exhaust valve
  Serial.println("Exhasting");
}
else
{
  digitalWrite(valve_1,0);
  digitalWrite(valve_2,0);
  digitalWrite(pump_A,0); //pump off
  digitalWrite(pump_B,0);
  Serial.println("System OFF");
}
}
