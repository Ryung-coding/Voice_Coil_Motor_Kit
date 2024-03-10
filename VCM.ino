#include "CytronMotorDriver.h" //motor driver header file
// Note* 만일 위 줄에서 오류가 날 경우, 함께 첨부한 라이브러리를 추가하시기 바랍니다.
// 아두이노 프로그램의 상단의 -> 툴 -> 라이브러리 포함하기 -> .zip 라이브러리 추가 -> 첨부된 zip 라이브러리 파일 선택

//Pin Map 
#define LVDT_pin A15      //    LVDT의 [Out+]를 보드의 [A15]에 연결
#define motor_pwm_pin 2   //motor driver의 [pwm]를 보드의 [2]에 연결
#define motor_dir_pin 3   //motor driver의 [Dir]를 보드의 [3]에 연결
CytronMD motor(PWM_DIR, motor_pwm_pin, motor_dir_pin); //motor driver 설정완료


//Calibation factor
#define analogData_to_distance_mm 3.67  //실험을 통하여 LVDT의 OUTPUT의 1 증가량에 따른 거리를 측정하여 mm 단위로 기입



//PID gain & controller gain (User Setting Point)--------------------------------------------------------------------------------------

double Kp = 0.85;                           // PID gain의 P 게인을 설정하시오 (최적값 0.85) [ ]
double Ki = 0.3;                            // PID gain의 P 게인을 설정하시오 (최적값 0.3)  [ ]
double Kd = 0.003;                          // PID gain의 P 게인을 설정하시오 (최적값 0.003)[ ]
double error_range = 3;                     // 허용 가능한 위치 오차를 설정하시오 (최적값 3)   [10^-6m]
double reference_running_time_ms = 8000;    // LVDT가 이동한 거리에서의 유지 시간을 설정하시오     [ms] ex) 8000 일 경우 입력하신 위치로 이동후, 8000ms 이후 복귀

//-------------------------------------------------------------------------------------------------------------------------------------


//Control value (don't touch)
double reference = 0; double reference_past = 0; 
double now_distance = 0; double past_distance = 0; 
int16_t u = 0; int16_t u_past = 0; int16_t y_past = 0; double I = 0;                                 
double millisTime_i; double millisTime_f; double dt = 0;                                
double LVDT_zero_point = 0; double zero_set_step = 10; 
double start_time = 0;


//-------------------------------------------------------------------------------------------------------------------------------------

//lowpassfilter() : LVDT에서 발생하는 잡음을 제거하여 반환
float lowpassfilter(float filter, float data, float lowpass_constant)
{
  filter = data * (1 - lowpass_constant) + filter * lowpass_constant;
  return filter;   
}

//-------------------------------------------------------------------------------------------------------------------------------------

//update_reference() : 사용자가 원하는 LVDT의 위치를 업데이트
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

//-------------------------------------------------------------------------------------------------------------------------------------

//sampling_time() : 보드의 실시간 작동시간을 연산하여 반환
double sampling_time()
{  
  millisTime_f = millis() ;
  double sampling_time_sec = (millisTime_f - millisTime_i)*0.001;
  millisTime_i = millis();
  return sampling_time_sec;
}

//-------------------------------------------------------------------------------------------------------------------------------------

//input_LVDT()() : LVDT의 입력값을 받아 mm 단위로 환산하여 반환
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

//-------------------------------------------------------------------------------------------------------------------------------------

//computePID() :PID 제어기를 작동하여 출력값을 반환
int16_t computePID(double r, double y, double dt)
{
  double error = y - r ;
  
  double P = Kp * error;  
  
  I +=Ki * error * dt;

  double D = Kd * -(y - y_past) / dt;
  y_past = y;
  
  u = abs(error) <= error_range ? u_past : P + I + D;
  u_past = u;
  return u;
}

//-------------------------------------------------------------------------------------------------------------------------------------

//Setting_Initial_Values() :보드및 LVDT의 초기값을 설정
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

//-------------------------------------------------------------------------------------------------------------------------------------

//Run_VCM()) : motor driver를 작동하여 VCM을 구동
void Run_VCM()
{
  motor.setSpeed(computePID(reference, now_distance, dt));
}

//-------------------------------------------------------------------------------------------------------------------------------------

//Data_Write() : 입력값, LVDT의 위치 등 얻은 값들을 확인
void Data_Write()
{
 Serial.print("입력값 : ");
 Serial.print(reference);
 Serial.print(" , ");
 Serial.print("100");
 Serial.print(" , ");
 Serial.print("-100");
 Serial.print(" , ");
 Serial.print("현재값 : ");
 Serial.println(now_distance);
}

//-------------------------------------------------------------------------------------------------------------------------------------
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
