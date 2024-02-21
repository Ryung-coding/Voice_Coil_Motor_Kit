




#define LVDT_pin A15
#define analogData_to_distance_mm 3.67
#include "CytronMotorDriver.h"

//PID gain & controller gain
double Kp = 0.8; double Ki = 0.3; double Kd = 0.00; double I = 0;
double I_max = 100000.0; double I_min = -1000000; double bangbang_control_range = 3;

double reference = 0; double reference_past = 0; double now_distance = 0; double past_distance = 0; double y_past = 0; int16_t u = 0; int16_t u_past = 0; 
                                 
double millisTime_i; double millisTime_f; double dt = 0;                                

double LVDT_zero_point = 0; double zero_set_step = 10; 
double reference_running_time_ms = 8000; double start_time = 0;

CytronMD motor(PWM_DIR, 2, 3);  // PWM = Pin 2, DIR = Pin 3.


float lowpassfilter(float filter, float data, float lowpass_constant)
{
  filter = data * (1 - lowpass_constant) + filter * lowpass_constant;
  return filter;   
}

void update_reference()
{
  int data = Serial.parseInt();
    
  if (data != 0) 
  {
    start_time = millis();
    reference = data;  
  }
  else 
  { 
    reference = reference_past;
  }
   if(start_time + reference_running_time_ms <= millis()) reference = 0;
   reference_past= reference;
}

double sampling_time()
{  
  millisTime_f = millis() ;
  double sampling_time_sec = (millisTime_f - millisTime_i)*0.001;
  millisTime_i = millis();
  return sampling_time_sec;
}

double input_LVDT()
{
  if(analogRead(LVDT_pin)<= 0 or analogRead(LVDT_pin)>= 1023)
  {
    while(1)
    {
      Serial.println("------- Out of bounds!! -------");
      Serial.println("Check : Check the location of the VCM again");
      delay(1000);
      if(analogRead(LVDT_pin)> 0 or analogRead(LVDT_pin)< 1024) break;
    }
  }

  double LVDT_raw_data = analogRead(LVDT_pin);
  double distance = (LVDT_raw_data - LVDT_zero_point) * analogData_to_distance_mm;
  distance = lowpassfilter(past_distance ,distance,0.99);
  return distance;
}

int16_t computePID(double r, double y, double dt)
{
  double error = y - r ;
  
  double P = Kp * error;  
  
  I +=Ki * error * dt;
  //I = constrain(I , I_min , I_max);
  //if(I == I_max ) I = (I_min+I_max)/2;
  
  double D = Kd * -(y - y_past) / dt;
  y_past = y;
  
  
  u = abs(error) <= bangbang_control_range ? u_past : P + I + D;
  u_past = u;
  return u;
}

void Setting_Initial_Values()
{ 
  Serial.println("------- Start the LVDT setup -------");
  double buf = 0;
  for(int i = 0 ; i < zero_set_step ;  i++)
  {
    if(analogRead(LVDT_pin)<= 0 or analogRead(LVDT_pin)>= 1023)
    {
      while(1)
      {
        Serial.println("------- Out of bounds!! -------");
        Serial.println("Check : Check the location of the VCM again");
        delay(1000);
        if(analogRead(LVDT_pin)> 0 or analogRead(LVDT_pin)< 1024) break;
      }
    }
    else 
    {
      Serial.print(" .");
      buf+=analogRead(LVDT_pin);
      delay(100);
    }
  }
  LVDT_zero_point = (buf/zero_set_step);
  Serial.print("\n");
  Serial.println("LVDT initial location calibration Done!!");
  Serial.print("Your Zero value is <");
  Serial.print(LVDT_zero_point);
  Serial.println(">.");
  delay(100);
}

void Run_VCM()
{
  motor.setSpeed(computePID(reference, now_distance, dt));
}


void Data_Write()
{
 Serial.print("ref : ");
 Serial.print(reference);
 Serial.print(",");
 Serial.print("100");
 Serial.print(",");
 Serial.print("-100");
 Serial.print(",");
 
 Serial.print(", dt : ");
 Serial.print(dt);
 Serial.print("sec, now_distance : ");
 Serial.println(now_distance);
 Serial.print("mm.  ");
 Serial.println(u);
}
void setup() 
{
  Serial.begin(115200);
  Serial.setTimeout(10);
  {
Serial.println("████████████████████████████████████████████████████████████");
Serial.println("████████████████████████████████████████████████████████████");
Serial.println("████████████████████████████████████████████████████████████");
Serial.println("────────────────────────────────────────────────────────────");
Serial.println("────────────────────────────────────────────────────────────");
Serial.println("───────────────────█─────█────███───█──────█────────────────");
Serial.println("────────────────────█───█───██──────██────██────────────────");
Serial.println("────────────────────█───█───█───────█─█──█─█────────────────");
Serial.println("────────────────────█───█───█───────█─█──█─█────────────────");
Serial.println("─────────────────────█─█────█───────█─█──█─█────────────────");
Serial.println("─────────────────────█─█────█───────█──██──█────────────────");
Serial.println("──────────────────────█──────████───█──██──█────────────────");
Serial.println("────────────────────────────────────────────────────────────");
Serial.println("──────────────────────────Voice Coil Motor Kit──────────────────────");
Serial.println("────────────────────────────────────────────────────────────");
Serial.println("████████████████████████████████████████████████████████████");
Serial.println("████████████████████████████████████████████████████████████");
  }
  int i=0;
  while(1)
  {
    
    Serial.println("█                          If you want to operate vcm, please type y                            █");
    Serial.println("████████████████████████████████████████████████████████████");
  }
  delay(1500);
  Setting_Initial_Values();
}


void loop() 
{ 
 update_reference();

 dt = sampling_time();
 now_distance = input_LVDT();
 past_distance = now_distance;
 Run_VCM();
}
