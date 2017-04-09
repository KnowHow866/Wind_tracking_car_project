#include <Servo.h>
const int default_pin = 13; //執行尋機運轉模式時會閃亮

//風扇馬達
const int sp1 = 12; //電變訊號接腳
const int motor_init_pin = 10;  //重新設定電變
int settingMode = 0; //重新設定電變的狀態
int motor_state = 0;  
int myangle1;
int pulsewidth;

//ＩＲ參數  
const int dR = 4;       int DR = 0; 
const int dM = 5;       int DM = 0; 
const int dL = 6;       int DL = 0; 
const int aR = 9;       int AR = 0;
const int aM = 10;      int AM = 0;
const int aL = 11;      int AL = 0;

//伺服馬達參數
const int servo_pin = 11;
Servo control;
int pos = 90;

//控制參數
int Kp = 15;
int err = 0;
int lastest_err = 0; //只能為正負一或零，處理嚴重偏移的例外狀況
int react = 0;
int delay_time = 100;

//開機與關機
int bootMode = 0;   //代表chmode_pin的狀態
volatile int start = 0;  //1時進入尋跡模式 
int stopping = 0;
int chmode_pin = 2;   //由這隻針腳讀入訊號來開機，需連接開關
int runtime = 5*1000;  int run_clock = runtime / delay_time;  int run_count = 0;//設置跑多久會自動關機
int notInterrupt_default = 0; //如果無法執行中斷，把它設為1

void setup() {
  //系統設定
  Serial.begin(9600);
  pinMode(default_pin, OUTPUT);
  pinMode(chmode_pin, INPUT);
  pinMode(motor_init_pin, INPUT);
  digitalWrite(chmode_pin, HIGH);
  digitalWrite(motor_init_pin, HIGH);

  //模式轉換的中斷綁定
  attachInterrupt( chmode_pin, chmode, CHANGE);

  //IR設置
  pinMode(dR, INPUT);
  pinMode(dM, INPUT);
  pinMode(dL, INPUT);

  //伺服馬達設置
  control.attach(servo_pin);
  control.write(pos);

  //無刷馬達設定
  pinMode(sp1,OUTPUT);
  Serial.println("servu=o_seral_simple ready" );
  //看电调说明书，设置油门行程时，一开始需要把遥控器打到最高点。i<=110大概是2杪多
  for(int i=0;i<=110;i++) { servopulse(sp1,150);}//引用脉冲函数 
  //等电机发出B-B两声后（就是两秒后，大概就是这里了）把油门打到最低点
  for(int i=0;i<=55;i++) { servopulse(sp1,20);}//引用脉冲函数 
  //后面提示后，就可以控制电机转动了
  for(int i=0;i<=150;i++) {servopulse(sp1,150);}//引用脉冲函数
  //pinMode(motor_init_pin, HIGH);
  Serial.println("servu=o_seral_simple OK" );
    
}

void loop() {

  //待機模式～～～～～～～～～～～～～～～
  Serial.println("Waiting mode");
  while(1){
    IR_read();
    //IR_print();
    delay(100);
    digitalWrite(default_pin , LOW);

    //一，電腦指令's'啟動
    //二，chmod_pin(2)的電壓狀態改變來開機
    //三，motor_init_pin(10)重新設定電變
    char cmd = Serial.read();
    enable();
    set();

    //進入循跡模式＿＿條件
    if(start == 1 || cmd == 's'){
      start = 0;
      fanMotor(1);
      motor_turn(0);
      Serial.println("System start");
      break;
    }
  }
  //～～～～～～～～～～～～～～～～～～～
  
  while(1){
  //--------------------循跡模式運轉------------------------------------- 
  //系統設定
  digitalWrite(default_pin, HIGH);
  IR_read();
  //IR_print();
  delay(delay_time - 20); //讀取速度 （扣掉油門航程脈衝)

  //控制器
  err -= DL;
  err += DR;
  if(DM == 0) err*=2;
  int react = err * Kp;
  
  //例外處理
  exeception_deal(); 

  //伺服馬達反應
  motor_turn(react);  

  //後設置
  if(react == 0) lastest_err = 0;   
  else if(react > 0) lastest_err = 1;
  else lastest_err = -1;
  err = 0; //清空儲存err的buffer

  //Serial.print("react: "); Serial.println(react);
  //Serial.println("");

  //補足馬力
  for(int i = 0; i < 4; i++){
    servopulse(sp1,100);
  }
  //系統性停車處理
  char man_stop = Serial.read();
  run_count ++; 
  if(man_stop == 'n' || run_count == run_clock) {
      fanMotor(0);
      run_count = 0;
      break;
  }
  //---------------------------------------------------------------------------
  }
}

//---------------------------FUNCTION-------------------------

//伺服馬達反應
void motor_turn(int react){
  int the_react = pos + react;
  control.write(the_react);
}

//IR讀取環境資料
void IR_read(){
  DR = digitalRead(dR);
  DM = digitalRead(dM);
  DL = digitalRead(dL);

  AR = analogRead(aR);
  AM = analogRead(aM);
  AL = analogRead(aL);
}

//IR輸出資料
void IR_print(){
  Serial.print("R: "); Serial.println(DR); //Serial.print(" , "); Serial.println(AR);
  Serial.print("M: "); Serial.println(DM); //Serial.print(" , "); Serial.println(AM);
  Serial.print("L: "); Serial.println(DL); //Serial.print(" , "); Serial.println(AL);
}

//啟動函數
void chmode(){
  start = 1;
  Serial.println("interrupt");
}

//例外處理
void exeception_deal(){
  if(DL == 1 && DR == 1)  {
    react = 0;//無法判定的狀況 but 停止區可能偵測到這個模式
    //Serial.println("left:1 & right:1");
  }
  else;
  
  if(err == 0 && DM == 0){  //嚴重的偏移
    react = lastest_err * Kp *3;
    //Serial.println("serious out");
  }
}

//風扇馬達控制
void fanMotor(int state){
  int val;
  
  if(state == 1)  val=3;
  else val = 0;

    if(val> 0 && val <= 9)
    {
      int val1 = val;//将特征量转化为数值变量
      val1=map(val1,0,9,0,180);//将角度转化为500-2480的脉宽值
      Serial.print("moving servo to ");
      Serial.print(val1,DEC);
      Serial.println();
      for(int i=0;i<=10;i++)//给予舵机足够的时间让它转到指定角度
      {
        servopulse(sp1,val1);//引用脉冲函数
      }
    }
  
}

//模擬油門航程
void servopulse(int sp1,int val1)//定义一个脉冲函数
{
  myangle1=map(val1,0,180,500,2480);
  digitalWrite(sp1,HIGH);//将舵机接口电平至高
  delayMicroseconds(myangle1);//延时脉宽值的微秒数
  digitalWrite(sp1,LOW);//将舵机接口电平至低
  delay(20-val1/1000);
}

//電變設定
void motor_init(){
  //看电调说明书，设置油门行程时，一开始需要把遥控器打到最高点。i<=110大概是2杪多
  for(int i=0;i<=110;i++)  servopulse(sp1,150);//引用脉冲函数 
  //等电机发出B-B两声后（就是两秒后，大概就是这里了）把油门打到最低点
  for(int i=0;i<=55;i++)  servopulse(sp1,20);//引用脉冲函数 
  //后面提示后，就可以控制电机转动了
  for(int i=0;i<=150;i++) servopulse(sp1,150);//引用脉冲函数
}

//讀取啟動針腳開機
void enable(){
  char cmd = Serial.read();
  int digital_cmd = digitalRead(chmode_pin); 
  if(digital_cmd == LOW && bootMode == 0){
    start = 1;  bootMode = 1;
  }
  else if(digital_cmd == HIGH && bootMode == 1){
    start = 1;  bootMode = 0;
  }
}

//重新設定電變
void set(){
  char init_pin = digitalRead(motor_init_pin);
  if(init_pin == LOW && settingMode == 0){
    settingMode = 1;
    motor_init();
    delay(1000);
  }
  else if(init_pin == HIGH && settingMode == 1){
    settingMode = 0;
    motor_init();
    delay(1000);
  }
}





