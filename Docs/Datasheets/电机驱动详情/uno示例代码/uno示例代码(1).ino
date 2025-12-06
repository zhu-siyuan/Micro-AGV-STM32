// 定义电机控制引脚
#define ENA 5   // 电机1 PWM调速引脚（必须支持PWM）
#define IN1 6   // 电机1方向引脚1
#define IN2 7   // 电机1方向引脚2
#define ENB 10  // 电机2 PWM调速引脚
#define IN3 8   // 电机2方向引脚1
#define IN4 9   // 电机2方向引脚2

void setup() {
  // 初始化所有引脚为输出
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("UNO两路电机控制示例");
}

void loop() {
  // 示例动作：电机1正转，电机2反转，然后停止
  setMotor(1, 255, true);  // 电机1全速正转
  setMotor(2, 150, false); // 电机2半速反转
  delay(2000);

  stopMotor(1);            // 停止电机1
  stopMotor(2);            // 停止电机2
  delay(1000);
}

/**
 * 控制指定电机运动
 * @param motor 电机编号（1或2）
 * @param speed 速度（0-255）
 * @param forward 方向（true=正转，false=反转）
 */
void setMotor(int motor, int speed, bool forward) {
  if (motor == 1) {
    analogWrite(ENA, speed);               // 设置电机1速度+
    digitalWrite(IN1, forward ? HIGH : LOW);
    digitalWrite(IN2, forward ? LOW : HIGH);
  } else if (motor == 2) {
    analogWrite(ENB, speed);               // 设置电机2速度
    digitalWrite(IN3, forward ? HIGH : LOW);
    digitalWrite(IN4, forward ? LOW : HIGH);
  }
}

/**
 * 停止指定电机
 */
void stopMotor(int motor) {
  if (motor == 1) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  } else if (motor == 2) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}