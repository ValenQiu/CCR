int step_pin = 4; // Enable
int dir_pin = 5; // Step
int enable_pin = 6; // Dir

int limit_A = 8;
int limit_B = 9;

int x;
bool limit_A_flag;

void setup()
{
  pinMode(enable_pin,OUTPUT); 
  pinMode(step_pin,OUTPUT); 
  pinMode(dir_pin,OUTPUT); 
  digitalWrite(enable_pin,HIGH); // Set Enable low

  pinMode(limit_A, INPUT);
  pinMode(limit_B, INPUT);

  Serial.begin(9600);
}

void loop()
{
  int limit_A_value = digitalRead(limit_A);
  limit_A_flag = (limit_A_value > 0);

  Serial.print("Sensor Value: ");
  Serial.println(limit_A_value);

  Serial.print("Sensor Flag: ");
  Serial.println(limit_A_flag);

  delay(200);
}
