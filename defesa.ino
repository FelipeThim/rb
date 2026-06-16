//==================== DEFINIÇÃO DOS MOTORES ====================

// Motor 1 - Traseira Direita
#define M1_PWM 3
#define M1_IN1 22
#define M1_IN2 23

// Motor 2 - Frente Direita
#define M2_PWM 2
#define M2_IN1 25
#define M2_IN2 24

// Motor 3 - Traseira Esquerda
#define M3_PWM 4
#define M3_IN1 47
#define M3_IN2 46

// Motor 4 - Frente Esquerda
#define M4_PWM 5
#define M4_IN1 49
#define M4_IN2 48

//==============================================================
// PWM INDIVIDUAL DE CADA MOTOR
// Ajuste estes valores para o robô andar reto
//==============================================================

int PWM_M1 = 200;
int PWM_M2 = 0;
int PWM_M3 = 0;
int PWM_M4 = 0;

//==============================================================

void configurar() {

  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M3_PWM, OUTPUT);
  pinMode(M4_PWM, OUTPUT);

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);

  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);

  pinMode(M4_IN1, OUTPUT);
  pinMode(M4_IN2, OUTPUT);
}

//==============================================================

void parar() {

  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
  analogWrite(M3_PWM, 0);
  analogWrite(M4_PWM, 0);

  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);

  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);

  digitalWrite(M3_IN1, LOW);
  digitalWrite(M3_IN2, LOW);

  digitalWrite(M4_IN1, LOW);
  digitalWrite(M4_IN2, LOW);
}

//==============================================================
// MOTOR 1
//==============================================================

void motor1Horario() {

  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);

  analogWrite(M1_PWM, PWM_M1);
}

void motor1AntiHorario() {

  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, HIGH);

  analogWrite(M1_PWM, PWM_M1);
}

//==============================================================
// MOTOR 2
//==============================================================

void motor2Horario() {

  digitalWrite(M2_IN1, HIGH);
  digitalWrite(M2_IN2, LOW);

  analogWrite(M2_PWM, PWM_M2);
}

void motor2AntiHorario() {

  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, HIGH);

  analogWrite(M2_PWM, PWM_M2);
}

//==============================================================
// MOTOR 3
//==============================================================

void motor3Horario() {

  digitalWrite(M3_IN1, HIGH);
  digitalWrite(M3_IN2, LOW);

  analogWrite(M3_PWM, PWM_M3);
}

void motor3AntiHorario() {

  digitalWrite(M3_IN1, LOW);
  digitalWrite(M3_IN2, HIGH);

  analogWrite(M3_PWM, PWM_M3);
}

//==============================================================
// MOTOR 4
//==============================================================

void motor4Horario() {

  digitalWrite(M4_IN1, HIGH);
  digitalWrite(M4_IN2, LOW);

  analogWrite(M4_PWM, PWM_M4);
}

void motor4AntiHorario() {

  digitalWrite(M4_IN1, LOW);
  digitalWrite(M4_IN2, HIGH);

  analogWrite(M4_PWM, PWM_M4);
}

//==============================================================
// MOVIMENTOS
//==============================================================

void frente() {

  parar();

  motor1Horario();
  motor2Horario();
  motor3AntiHorario();
  motor4Horario();
}

void tras() {

  parar();

  motor1AntiHorario();
  motor2AntiHorario();
  motor3Horario();
  motor4AntiHorario();
}

void direita() {

  parar();

  motor1AntiHorario();
  motor2AntiHorario();

  motor3Horario();
  motor4Horario();
}

void esquerda() {

  parar();

  motor1Horario();
  motor2Horario();

  motor3AntiHorario();
  motor4AntiHorario();
}

//==============================================================

void setup() {

  Serial.begin(115200);

  configurar();

  parar();

  Serial.println();
  Serial.println("=======================================");
  Serial.println(" CONTROLE DO ROBO");
  Serial.println("=======================================");
  Serial.println("f -> Frente");
  Serial.println("t -> Tras");
  Serial.println("d -> Girar Direita");
  Serial.println("e -> Girar Esquerda");
  Serial.println("p -> Parar");
  Serial.println();
  Serial.println("PWM atuais:");
  Serial.print("Motor 1 = "); Serial.println(PWM_M1);
  Serial.print("Motor 2 = "); Serial.println(PWM_M2);
  Serial.print("Motor 3 = "); Serial.println(PWM_M3);
  Serial.print("Motor 4 = "); Serial.println(PWM_M4);
}

//==============================================================

void loop() {

  if (Serial.available()) {

    char c = Serial.read();

    switch (c) {

      case 'f':
        Serial.println("FRENTE");
        frente();
        break;

      case 't':
        Serial.println("TRAS");
        tras();
        break;

      case 'd':
        Serial.println("DIREITA");
        direita();
        break;

      case 'e':
        Serial.println("ESQUERDA");
        esquerda();
        break;

      case 'p':
        Serial.println("PARAR");
        parar();
        break;
    }
  }
}
