// Pinout Definition
int step_pin = 4; // Enable
int dir_pin = 5; // Step
int enable_pin = 6; // Dir
int limit_A = 8;
int limit_B = 9;

// Global variables
int x;    // To count the steps in every motion
int pos;  // To record the position, 400 steps per round, each round is 4mm travel
int limit_A_value;
bool limit_A_flag;
int limit_B_value;
bool limit_B_flag;

void setup() {
  // put your setup code here, to run once:
  pinMode(enable_pin,OUTPUT); 
  pinMode(step_pin,OUTPUT); 
  pinMode(dir_pin,OUTPUT); 
  
  digitalWrite(enable_pin,HIGH); // Set Enable low
  digitalWrite(step_pin, LOW);
  digitalWrite(dir_pin, LOW);

  init_pos();
  }

void init_pos()
{
  limit_A_flag = read_limit(limit_A);     
  limit_B_flag = read_limit(limit_B); 

  if (limit_A_flag == true && limit_B_flag== true)   // In the middle
  {
    digitalWrite(dir_pin,HIGH); // Set Dir high
    while (limit_A_flag) 
    {
      digitalWrite(step_pin,HIGH); // Output high
      delayMicroseconds(800); // Wait 1/2 a ms
      digitalWrite(step_pin,LOW); // Output low
      delayMicroseconds(800); // Wait 1/2 a ms                   // Move one step in the clockwise direction
      // Update the reading of sensor A
      limit_A_flag = read_limit(limit_A);
    }
  }
  else if (limit_A_flag == false && limit_B_flag == true)  // In limit position A
  {
    digitalWrite(dir_pin,LOW); // Set Dir low
    for(int x = 0; x < 1000; x++)
    {
      digitalWrite(step_pin,HIGH); // Output high
      delayMicroseconds(800); // Wait 1/2 a ms
      digitalWrite(step_pin,LOW); // Output low
      delayMicroseconds(800); // Wait 1/2 a ms    
    }
    limit_A_flag = read_limit(limit_A);
    digitalWrite(dir_pin,HIGH); // Set Dir high
    while (limit_A_flag) 
    {
      digitalWrite(step_pin,HIGH); // Output high
      delayMicroseconds(800); // Wait 1/2 a ms
      digitalWrite(step_pin,LOW); // Output low
      delayMicroseconds(800); // Wait 1/2 a ms                   // Move one step in the clockwise direction
      // Update the reading of sensor A
      limit_A_flag = read_limit(limit_A);
    }
  }
  pos = 0;
}

bool read_limit(int id)
{
  int limit_value = digitalRead(id);
  bool limit_flag = (limit_value > 0);   // If the limit sensor is open, returns True
  return limit_flag;
}

void stop()
{
  digitalWrite(dir_pin, LOW);
  digitalWrite(step_pin, LOW);
}

int counterclockwise(int step)
{ 
  digitalWrite(dir_pin,LOW); // Set Dir low

  for(x = 0; x < step; x++) // Loop 200 times
  {   
      limit_B_flag = read_limit(limit_B);
      if (limit_B_flag == false)
      {
        return pos += x;
      }
      digitalWrite(step_pin,HIGH); // Output high
      delayMicroseconds(800);
      digitalWrite(step_pin,LOW); // Output low
      delayMicroseconds(800);
    }
  return pos += x;
}

int clockwise(int step)
{
  digitalWrite(dir_pin,HIGH); // Set Dir high

  for(x = 0; x < step; x++) // Loop 200 times
  {
    limit_A_flag = read_limit(limit_A);
    if (limit_A_flag == false)
    {
      return pos -= x
    }
    digitalWrite(step_pin,HIGH); // Output high
    delayMicroseconds(800); // Wait 1/2 a ms
    digitalWrite(step_pin,LOW); // Output low
    delayMicroseconds(800); // Wait 1/2 a ms
  }
  return pos -= x;
}

void loop() {
  // put your main code here, to run repeatedly:
//  clockwise(4000);
//  delay(1000);
  counterclockwise(4000);
  delay(1000);
}
