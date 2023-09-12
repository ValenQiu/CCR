#define HX710_SCK 2
#define HX710_DT 3
long HX710_Buffer = 0;
long Pressure_rough = 0,Pressure_read = 0;
//****************************************************
//Initialize HX711
//****************************************************
void Init_Hx710(){
  pinMode(HX710_SCK, OUTPUT); 
  pinMode(HX710_DT, INPUT);
}
//****************************************************
//Read the rough pressure
//****************************************************
void Get_Rough()
{
  HX710_Buffer = HX710_Read();
  Pressure_rough = HX710_Buffer/100;    
} 

//****************************************************
//Measure the pressure
//****************************************************
unsigned int Get_Pressure()
{
  HX710_Buffer = HX710_Read();
  HX710_Buffer = HX710_Buffer/100;

  Pressure_read = HX710_Buffer;
  Pressure_read = Pressure_read - Pressure_rough;       //read the actual AD sampling value
  
  Pressure_read = (unsigned int)((float)Pressure_read/7.35);   
    //Calculating the actual pressure
    //Since the difference of different sensor, thus, ever sensor need to fix the divisor 
    //当发现测试出来的重量偏大时，增加该数值。
    //如果测试出来的重量偏小时，减小改数值。
    //该数值一般在7.16左右。因传感器不同而定。
    //+0.05是为了四舍五入百分位

  return Pressure_read;
}

//****************************************************
//读取HX711
//****************************************************
unsigned long HX710_Read(void)  //增益128
{
  unsigned long count; 
  unsigned char i;
  bool Flag = 0;
  digitalWrite(HX710_DT, HIGH);
  delayMicroseconds(1);
  digitalWrite(HX710_SCK, LOW);
  delayMicroseconds(1);
  count=0; 
  while(digitalRead(HX710_DT)); 
  for(i=0;i<24;i++)
  { 
    digitalWrite(HX710_SCK, HIGH); 
    delayMicroseconds(1);
    count=count<<1; 
    digitalWrite(HX710_SCK, LOW); 
    delayMicroseconds(1);
    if(digitalRead(HX710_DT))
      count++; 
  } 
  digitalWrite(HX710_SCK, HIGH); 
  count ^= 0x800000;
  delayMicroseconds(1);
  digitalWrite(HX710_SCK, LOW); 
  delayMicroseconds(1);
  
  return(count);
}
