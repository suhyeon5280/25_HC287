// === MDD20A 모터 드라이버 핀 ===
#define MDD20A_pwmR 9   // 오른쪽 모터 PWM (A)
#define MDD20A_dirR 12  // 오른쪽 모터 방향 (A)
#define MDD20A_pwmL 11  // 왼쪽 모터 PWM (B)
#define MDD20A_dirL 13  // 왼쪽 모터 방향 (B)

// ★★★ 추가: 엔코더 핀 (실제 연결된 핀으로 수정 필수) ★★★
// 아두이노 우노/나노의 경우 인터럽트 핀은 2, 3번 입니다.
#define ENCODER_R_A 2 // 오른쪽 엔코더 A상 (인터럽트 0)
#define ENCODER_R_B 4 // 오른쪽 엔코더 B상
#define ENCODER_L_A 3 // 왼쪽 엔코더 A상 (인터럽트 1)
#define ENCODER_L_B 5 // 왼쪽 엔코더 B상

// ★★★ 추가: 엔코더 틱 카운트 변수 ★★★
// 인터럽트 서비스 루틴(ISR)에서 사용되므로 volatile 키워드 필수
volatile long tick_r = 0;
volatile long tick_l = 0;

// 시리얼 통신 관련 변수
char serial_buffer[32];
uint8_t buffer_idx = 0;

// 타이머 변수
unsigned long last_cmd_time = 0;
unsigned long last_encoder_time = 0;

// 모터 PWM 값
int pwm_r = 0;
int pwm_l = 0;

// 속도 보정 계수 (로봇에 맞게 튜닝 필요)
const double gain = 200;

void setup() {
  // 모터 드라이버 핀 출력으로 설정
  pinMode(MDD20A_pwmL, OUTPUT);
  pinMode(MDD20A_pwmR, OUTPUT);
  pinMode(MDD20A_dirL, OUTPUT);
  pinMode(MDD20A_dirR, OUTPUT);

  // ★★★ 추가: 엔코더 핀 입력으로 설정 ★★★
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);
  
  // ★★★ 추가: 엔코더 인터럽트 설정 ★★★
  // A상 핀의 신호가 바뀔 때마다(RISING) 지정된 함수를 실행
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), handle_right_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), handle_left_encoder, RISING);

  Serial.begin(115200);
}

void loop() {
  // ===== 시리얼 명령 수신 및 파싱 =====
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      serial_buffer[buffer_idx] = '\0';
      parseSerialCommand(serial_buffer);
      buffer_idx = 0;
    } else if (buffer_idx < sizeof(serial_buffer) - 1) {
      serial_buffer[buffer_idx++] = c;
    }
  }

  // ===== 타임아웃 시 모터 정지 (수정된 부분) =====
  /*
  if (millis() - last_cmd_time > 500) {
    pwm_r = 0;
    pwm_l = 0;
  }
  */

  // ===== 모터 구동 =====
  drive_motors();
  
  // ★★★ 추가: 주기적으로 엔코더 데이터 PC로 전송 ★★★
  if (millis() - last_encoder_time > 20) { // 50Hz (20ms 마다)
    Serial.print("e,");
    Serial.print(-tick_r);
    Serial.print(",");
    Serial.print(-tick_l);
    Serial.println(",");
    last_encoder_time = millis();
  }
}

void parseSerialCommand(char *cmd) {
  // PC로부터 받은 속도 명령 파싱 (예: "s,0.50,-0.30\n")
  if (cmd[0] == 's') {
    char *token = strtok(cmd + 2, ",");
    if (token != NULL) {
      double vl = atof(token); // 왼쪽 바퀴 속도 (m/s)
      token = strtok(NULL, ",");
      if (token != NULL) {
        double vr = atof(token); // 오른쪽 바퀴 속도 (m/s)

        // 속도(m/s)를 PWM 값으로 변환
        pwm_l = vl * gain;
        pwm_r = vr * gain;
      }
    }
  }
  last_cmd_time = millis();
}

void drive_motors() {
  // 오른쪽 모터 방향 및 속도 제어
  if (pwm_r >= 0) digitalWrite(MDD20A_dirR, HIGH);
  else digitalWrite(MDD20A_dirR, LOW);
  analogWrite(MDD20A_pwmR, constrain(abs(pwm_r), 0, 255));
  
  // 왼쪽 모터 방향 및 속도 제어
  if (pwm_l >= 0) digitalWrite(MDD20A_dirL, LOW);
  else digitalWrite(MDD20A_dirL, HIGH);
  analogWrite(MDD20A_pwmL, constrain(abs(pwm_l), 0, 255));
}

// ★★★ 추가: 오른쪽 엔코더 인터럽트 처리 함수 ★★★
void handle_right_encoder() {
  if (digitalRead(ENCODER_R_B) == HIGH) {
    tick_r++; // 정방향
  } else {
    tick_r--; // 역방향
  }
}

// ★★★ 추가: 왼쪽 엔코더 인터럽트 처리 함수 ★★★
void handle_left_encoder() {
  if (digitalRead(ENCODER_L_B) == LOW) { // 왼쪽은 반대 방향일 수 있음
    tick_l++; // 정방향
  } else {
    tick_l--; // 역방향
  }
}
