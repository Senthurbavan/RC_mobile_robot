
#define LOW_SPEED 1000.0
#define HIGH_SPEED 1980.0
#define CENTER_SPEED 1500.0

#define LOW_STEER 1000.0
#define HIGH_STEER 2000.0
#define CENTER_STEER 1500.0

#define ROTATE_EXTRA_SPEED 50

#define MAX_SCALE 0.85
#define ERROR_GAP_SPEED  80
#define ERROR_GAP_STEER  65 

#define PWM_MIN 30
#define PWM_MAX 255

#define ENR  6
#define ENL  7
#define INR1 30
#define INR2 31
#define INL1 32
#define INL2 33

#define SPEED_PIN 2
#define STEERING_PIN 3

#define SPEED_FLAG 1
#define STEERING_FLAG 2

#define ROBOT_STOP 0
#define ROBOT_FORWARD 1
#define ROBOT_REVERSE 2
#define ROBOT_ROTATE_RIGHT 3
#define ROBOT_ROTATE_LEFT 4

unsigned direct = ROBOT_STOP;
unsigned old_direct = ROBOT_STOP;

uint32_t speed_Start;
uint32_t steering_Start;

int pwm = 0;
int pwml = 0;
int pwmr = 0;

float scale ;

volatile uint8_t update_Flags_Shared;
volatile uint16_t speed_Shared;
volatile uint16_t steering_Shared;

void setup() {

  attachInterrupt(digitalPinToInterrupt(SPEED_PIN),calc_Speed,CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEERING_PIN),calc_Steering,CHANGE);
  pinMode(ENL,OUTPUT);
  pinMode(ENR,OUTPUT);
  pinMode(INR1,OUTPUT);
  pinMode(INR2,OUTPUT);
  pinMode(INL1,OUTPUT);
  pinMode(INL2,OUTPUT);
  Serial.begin(115200);

}

void loop() {
  static uint8_t update_Flags;
  static uint16_t speed_Val;
  static uint16_t steering_Val;

  if(update_Flags_Shared){
      noInterrupts();

      update_Flags = update_Flags_Shared;

      if(update_Flags & SPEED_FLAG){
          speed_Val = speed_Shared;
      }
      if(update_Flags & STEERING_FLAG){
          steering_Val = steering_Shared;  
      }

      update_Flags_Shared = 0;
      interrupts();     
  }

  if(update_Flags & SPEED_FLAG){
      speed_Val = constrain(speed_Val, LOW_SPEED, HIGH_SPEED);
      if(speed_Val >= (CENTER_SPEED + ERROR_GAP_SPEED )){
      scale = ((MAX_SCALE-1.0)/(HIGH_SPEED - CENTER_SPEED))*(speed_Val - CENTER_SPEED) + 1.0;
      direct = ROBOT_FORWARD;
      pwm = map(speed_Val,CENTER_SPEED,HIGH_SPEED,PWM_MIN,PWM_MAX);
      
   }else if(speed_Val <= (CENTER_SPEED - ERROR_GAP_SPEED )){
      scale = ((MAX_SCALE-1.0)/(CENTER_SPEED - LOW_SPEED))*(CENTER_SPEED - speed_Val) + 1.0;
      direct = ROBOT_REVERSE;
      pwm = map(speed_Val,CENTER_SPEED,LOW_SPEED,PWM_MIN,PWM_MAX);
   }else{
      scale = 0; 
      pwm = PWM_MAX; 
   }
  }

  if(update_Flags & STEERING_FLAG){
      steering_Val = constrain(steering_Val, LOW_STEER, HIGH_STEER);
      if(steering_Val >= (CENTER_STEER + ERROR_GAP_STEER)){
          if(scale == 0){
            direct = ROBOT_ROTATE_RIGHT; 
            pwml=pwmr = map(steering_Val,CENTER_STEER,HIGH_STEER,PWM_MIN+60,PWM_MAX-55);
          }else{
            //pwm += ROTATE_EXTRA_SPEED;
            pwml = pwm;
            pwmr = map(steering_Val,CENTER_STEER,HIGH_STEER,pwm,(pwm-(pwm*scale)));
          }
          
      }else if(steering_Val <= (CENTER_STEER - ERROR_GAP_STEER)){
          if(scale == 0){
            direct = ROBOT_ROTATE_LEFT;
            pwml=pwmr = map(steering_Val,CENTER_STEER,LOW_STEER,PWM_MIN+60,PWM_MAX-55);
          }else{
           // pwm += ROTATE_EXTRA_SPEED;
            pwmr = pwm;
            pwml = map(steering_Val,CENTER_STEER,LOW_STEER,pwm,(pwm-(pwm*scale)));
          }
          
      }else{
          pwml = pwmr = pwm;
          if(scale == 0){
            direct = ROBOT_STOP;
          }
               
      }
  }

  if(direct != old_direct){
          old_direct = direct;
          switch(direct){
            case ROBOT_FORWARD :
                digitalWrite(INR1,HIGH);
                digitalWrite(INR2,LOW);
                digitalWrite(INL1,HIGH);
                digitalWrite(INL2,LOW); 
                break;
            case ROBOT_REVERSE : 
                digitalWrite(INR1,LOW);
                digitalWrite(INR2,HIGH);
                digitalWrite(INL1,LOW);
                digitalWrite(INL2,HIGH);
                break;
            case ROBOT_ROTATE_RIGHT : 
                digitalWrite(INR1,LOW);
                digitalWrite(INR2,HIGH);
                digitalWrite(INL1,HIGH);
                digitalWrite(INL2,LOW);
                break;
            case ROBOT_ROTATE_LEFT :
                digitalWrite(INR1,HIGH);
                digitalWrite(INR2,LOW);
                digitalWrite(INL1,LOW);
                digitalWrite(INL2,HIGH); 
                break;
            case ROBOT_STOP : 
                digitalWrite(INR1,HIGH);
                digitalWrite(INR2,HIGH);
                digitalWrite(INL1,HIGH);
                digitalWrite(INL2,HIGH);
                break;
              
          }
  }

  //pwmr = constrain(pwmr, PWM_MIN, PWM_MAX);
  //pwml = constrain(pwml, PWM_MIN, PWM_MAX);
  
  analogWrite(ENR,pwmr);
  analogWrite(ENL,pwml);
  
   Serial.print(pwml);
   Serial.print("\t");
   Serial.print(pwmr);
   Serial.print("\t");
   Serial.println(scale);
  

  update_Flags = 0;
}




void calc_Speed(){
  if(digitalRead(SPEED_PIN) == HIGH){
      speed_Start = micros();
  }else{
      speed_Shared = (uint16_t)(micros() - speed_Start);
      update_Flags_Shared |= SPEED_FLAG;
  } 
}

void calc_Steering(){
   if(digitalRead(STEERING_PIN) == HIGH){
      steering_Start = micros();
  }else{
      steering_Shared = (uint16_t)(micros() - steering_Start);
      update_Flags_Shared |= STEERING_FLAG;
  } 
}
