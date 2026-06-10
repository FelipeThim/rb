// ----------------------- //

// Include libys:
#include <Wire.h>
#include <HTInfraredSeeker.h>
#include <string.h>

// ----------------------- //

// Ph_1 == Left Ponte H
// Ph_2 == Right Ponte H

// Define Ponte H 1:
// Motor_1 == Front Left
// Motor_2 == Back Left
#define in_1_ph_1 25 // Motor_2
#define in_2_ph_1 24 // Motor_2
#define in_3_ph_1 23 // Motor_1
#define in_4_ph_1 22 // Motor_1

// Define Ponte H 2:
// Motor_3 == Back Right
// Motor_4 == Front Right
#define in_1_ph_2 47 // Motor_4
#define in_2_ph_2 46 // Motor_4
#define in_3_ph_2 49 // Motor_3
#define in_4_ph_2 48 // Motor_3

// ----------------------- //

// PWM:
#define pwm_1 2 // Motor_1
#define pwm_2 3 // Motor_2
#define pwm_3 4 // Motor_3
#define pwm_4 5 // Motor_4

// ----------------------- //

// MUX pins:
int s0 = 8;
int s1 = 9;
int s2 = 10;
int s3 = 11;

int SIG_pin = A7;

// ----------------------- //

// Line sensor values:
int valorMin[16];
int valorMax[16];
int limiar[16];

bool sensorCalibrado[16];
bool linha[16];

const int TEMPO_CALIBRACAO = 6000;
const int VARIACAO_MINIMA = 80;

// ----------------------- //

bool habilitar_debug = true;

// ----------------------- //

// Direction of the ball
int ballDirecao = 0;
// Intensity of the ball  
int ballIntens = 0;

const int BALL_CENTER = 5;

// ----------------------- //

// Speeds:
const int FORWARD_SPEED = 160;
const int BACK_SPEED = 150;
const int SIDE_SPEED = 140;
const int SEARCH_SPEED = 120;

// ----------------------- //

// Motor direction:
const int HORARIO = 1;
const int ANTI_HORARIO = -1;
const int PARADO = 0;

// ----------------------- //

// Movement debug:
const char* ultimoMovimento = "";

// ----------------------- //
// Function prototypes

void calibrar();
void atualizar_sensores_linha();
void imprimir_sensores_linha();
void controle_linha_e_bola();
bool esta_na_posicao_certa_da_linha();
bool linha_na_frente();
bool linha_em_lugar_errado();
void seguir_bola_eixo_x();
void ir_reader();

int mediaLeituras(int canal);
int readMux(int channel);

void debug_msg(String msg);
void print_movimento(const char* movimento);

void motor_1(int sentido, int speed);
void motor_2(int sentido, int speed);
void motor_3(int sentido, int speed);
void motor_4(int sentido, int speed);

void move_foward(int speed);
void move_back(int speed);
void right_rotation(int speed);
void left_rotation(int speed);
void move_right(int speed);
void move_left(int speed);
void stop_robot();

// ----------------------- //

void setup() {
  Serial.begin(115200);

  pinMode(in_1_ph_1, OUTPUT); pinMode(in_2_ph_1, OUTPUT);
  pinMode(in_3_ph_1, OUTPUT); pinMode(in_4_ph_1, OUTPUT);

  pinMode(in_1_ph_2, OUTPUT); pinMode(in_2_ph_2, OUTPUT);
  pinMode(in_3_ph_2, OUTPUT); pinMode(in_4_ph_2, OUTPUT);

  pinMode(pwm_1, OUTPUT); pinMode(pwm_2, OUTPUT);
  pinMode(pwm_3, OUTPUT); pinMode(pwm_4, OUTPUT);

  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);

  Wire.begin();
  InfraredSeeker::Initialize();

  Serial.println("=== ROBO SOCCER COM LINHA ===");
  Serial.println("=== CALIBRACAO DINAMICA ===");

  calibrar();

  Serial.println("Calibracao concluida!");
  debug_msg("Robo iniciado!");
}

// ----------------------- //

void loop() {
  atualizar_gyro();
  ir_reader();
  monitor_serial();

  controle_goleiro_com_ldr();

  delay(25);
}

// ----------------------- //

void controle_linha_e_bola() {
  if (esta_na_posicao_certa_da_linha()) {
    if (habilitar_debug) {
      Serial.println("ESTADO: Linha certa -> seguindo bola no eixo X");
    }

    seguir_bola_eixo_x();
    return;
  }

  if (linha_na_frente()) {
    if (habilitar_debug) {
      Serial.println("ESTADO: Linha na frente -> indo para frente");
    }

    move_foward(FORWARD_SPEED);
    return;
  }

  if (linha_em_lugar_errado()) {
    if (habilitar_debug) {
      Serial.println("ESTADO: Linha em lugar errado -> indo para tras");
    }

    move_back(BACK_SPEED);
    return;
  }

  if (habilitar_debug) {
    Serial.println("ESTADO: Nenhuma linha detectada -> parado");
  }

  stop_robot();
}

// ----------------------- //

// The robot is correct when it has:
// S3 or S4 on one side
// and S14 or S15 on the other side
bool esta_na_posicao_certa_da_linha() {
  bool ladoDireito = linha[2] || linha[3];    // S3 or S4
  bool ladoEsquerdo = linha[13] || linha[14]; // S14 or S15

  return ladoDireito && ladoEsquerdo;
}

// ----------------------- //

// Front sensors:
// S1, S2 or S16
bool linha_na_frente() {
  return linha[0] || linha[1] || linha[15];
}

// ----------------------- //

// Wrong sensors:
// Any sensor that is not S1, S2, S3, S4, S14, S15 or S16
bool linha_em_lugar_errado() {
  for (int i = 0; i < 16; i++) {
    if (linha[i]) {
      if (i == 0 || i == 1 || i == 2 || i == 3 || i == 13 || i == 14 || i == 15) {
        continue;
      }

      return true;
    }
  }

  return false;
}

// ----------------------- //

// Follow ball only in X axis:
// Robot does not go forward here.
// It only moves left or right.
void seguir_bola_eixo_x() {
  if (ballDirecao == 0) {
    if (habilitar_debug) {
      Serial.println("BOLA: Nao encontrada -> parado");
    }

    stop_robot();
    return;
  }

  if (ballDirecao >= 4 && ballDirecao <= 6) {
    if (habilitar_debug) {
      Serial.println("BOLA: Centralizada -> parado");
    }

    stop_robot();
    return;
  }

  if (ballDirecao < BALL_CENTER) {
    if (habilitar_debug) {
      Serial.println("BOLA: Esquerda -> mover esquerda");
    }

    move_left(SIDE_SPEED);
    return;
  }

  if (ballDirecao > BALL_CENTER) {
    if (habilitar_debug) {
      Serial.println("BOLA: Direita -> mover direita");
    }

    move_right(SIDE_SPEED);
    return;
  }
}

// ----------------------- //

// Void ir_reader
void ir_reader() {
  InfraredResult InfraredBall = InfraredSeeker::ReadAC();

  ballDirecao = InfraredBall.Direction;
  ballIntens = InfraredBall.Strength;

  if (habilitar_debug) {
    Serial.print("Bola Direcao: ");
    Serial.print(ballDirecao);
    Serial.print("\tIntensidade: ");
    Serial.println(ballIntens);
  }
}

// ----------------------- //

// Update all line sensors
void atualizar_sensores_linha() {
  for (int i = 0; i < 16; i++) {
    int leitura = mediaLeituras(i);

    if (!sensorCalibrado[i]) {
      linha[i] = false;
      continue;
    }

    bool brancoEhMax = valorMax[i] > valorMin[i];

    if (brancoEhMax) {
      linha[i] = leitura >= limiar[i];
    } else {
      linha[i] = leitura <= limiar[i];
    }
  }
}

// ----------------------- //

// Print line sensors
void imprimir_sensores_linha() {
  Serial.print("Linha: ");

  for (int i = 0; i < 16; i++) {
    if (linha[i]) {
      Serial.print("S");
      Serial.print(i + 1);
      Serial.print(" ");
    }
  }

  Serial.println();
}

// ----------------------- //

// Dynamic calibration
// Move the robot over the line during calibration.
// The robot will detect which sensors saw a big variation.
void calibrar() {
  Serial.println("Coloque o robo perto da linha.");
  Serial.println("Durante a calibracao, mova o robo para alguns sensores passarem no branco.");
  Serial.println("Digite 1 para comecar.");

  while (true) {
    if (Serial.available()) {
      int v = Serial.parseInt();
      if (v == 1) break;
    }
  }

  for (int i = 0; i < 16; i++) {
    valorMin[i] = 1023;
    valorMax[i] = 0;
    sensorCalibrado[i] = false;
  }

  Serial.println("Calibrando... mova o robo sobre a linha.");

  unsigned long inicio = millis();

  while (millis() - inicio < TEMPO_CALIBRACAO) {
    for (int i = 0; i < 16; i++) {
      int leitura = mediaLeituras(i);

      if (leitura < valorMin[i]) {
        valorMin[i] = leitura;
      }

      if (leitura > valorMax[i]) {
        valorMax[i] = leitura;
      }
    }

    delay(20);
  }

  Serial.println("\n=== RESULTADO CALIBRACAO ===");

  for (int i = 0; i < 16; i++) {
    int variacao = valorMax[i] - valorMin[i];

    limiar[i] = (valorMin[i] + valorMax[i]) / 2;

    if (variacao >= VARIACAO_MINIMA) {
      sensorCalibrado[i] = true;
    } else {
      sensorCalibrado[i] = false;
    }

    Serial.print("S");
    Serial.print(i + 1);
    Serial.print(" Min:");
    Serial.print(valorMin[i]);
    Serial.print(" Max:");
    Serial.print(valorMax[i]);
    Serial.print(" Var:");
    Serial.print(variacao);
    Serial.print(" L:");
    Serial.print(limiar[i]);
    Serial.print(" Status:");

    if (sensorCalibrado[i]) {
      Serial.println(" OK");
    } else {
      Serial.println(" SEM_VARIACAO");
    }
  }

  Serial.println("Calibracao concluida!");
}

// ----------------------- //

// Average of readings
int mediaLeituras(int canal) {
  long soma = 0;

  for (int i = 0; i < 5; i++) {
    soma += readMux(canal);
    delay(2);
  }

  return soma / 5;
}

// ----------------------- //

// MUX reading
int readMux(int channel) {
  digitalWrite(s0, bitRead(channel, 0));
  digitalWrite(s1, bitRead(channel, 1));
  digitalWrite(s2, bitRead(channel, 2));
  digitalWrite(s3, bitRead(channel, 3));

  delayMicroseconds(50);

  return analogRead(SIG_pin);
}

// ----------------------- //

void debug_msg(String msg) {
  if (habilitar_debug) {
    Serial.println(msg);
  }
}

// ----------------------- //

void print_movimento(const char* movimento) {
  if (!habilitar_debug) {
    return;
  }

  if (strcmp(movimento, ultimoMovimento) != 0) {
    Serial.print("MOVIMENTO: ");
    Serial.println(movimento);
    ultimoMovimento = movimento;
  }
}

// ----------------------- //
// Motors

void motor_1(int sentido, int speed) {
  if (sentido == HORARIO) {
    digitalWrite(in_4_ph_1, LOW);
    digitalWrite(in_3_ph_1, HIGH);
  } else if (sentido == ANTI_HORARIO) {
    digitalWrite(in_4_ph_1, HIGH);
    digitalWrite(in_3_ph_1, LOW);
  } else {
    digitalWrite(in_4_ph_1, LOW);
    digitalWrite(in_3_ph_1, LOW);
  }

  analogWrite(pwm_1, speed);
}

void motor_2(int sentido, int speed) {
  if (sentido == HORARIO) {
    digitalWrite(in_1_ph_1, LOW);
    digitalWrite(in_2_ph_1, HIGH);
  } else if (sentido == ANTI_HORARIO) {
    digitalWrite(in_1_ph_1, HIGH);
    digitalWrite(in_2_ph_1, LOW);
  } else {
    digitalWrite(in_1_ph_1, LOW);
    digitalWrite(in_2_ph_1, LOW);
  }

  analogWrite(pwm_2, speed);
}

void motor_3(int sentido, int speed) {
  if (sentido == HORARIO) {
    digitalWrite(in_3_ph_2, HIGH);
    digitalWrite(in_4_ph_2, LOW);
  } else if (sentido == ANTI_HORARIO) {
    digitalWrite(in_3_ph_2, LOW);
    digitalWrite(in_4_ph_2, HIGH);
  } else {
    digitalWrite(in_3_ph_2, LOW);
    digitalWrite(in_4_ph_2, LOW);
  }

  analogWrite(pwm_3, speed);
}

void motor_4(int sentido, int speed) {
  if (sentido == HORARIO) {
    digitalWrite(in_1_ph_2, LOW);
    digitalWrite(in_2_ph_2, HIGH);
  } else if (sentido == ANTI_HORARIO) {
    digitalWrite(in_1_ph_2, HIGH);
    digitalWrite(in_2_ph_2, LOW);
  } else {
    digitalWrite(in_1_ph_2, LOW);
    digitalWrite(in_2_ph_2, LOW);
  }

  analogWrite(pwm_4, speed);
}

// ----------------------- //
// Movements

void move_foward(int speed) {
  print_movimento("FRENTE");

  motor_1(ANTI_HORARIO, speed);
  motor_2(ANTI_HORARIO, speed);
  motor_3(HORARIO, speed);
  motor_4(HORARIO, speed);
}

void move_back(int speed) {
  print_movimento("TRAS");

  motor_1(HORARIO, speed);
  motor_2(HORARIO, speed);
  motor_3(ANTI_HORARIO, speed);
  motor_4(ANTI_HORARIO, speed);
}

void right_rotation(int speed) {
  print_movimento("ROTACIONAR DIREITA");

  motor_1(ANTI_HORARIO, speed);
  motor_2(ANTI_HORARIO, speed);
  motor_3(ANTI_HORARIO, speed);
  motor_4(ANTI_HORARIO, speed);
}

void left_rotation(int speed) {
  print_movimento("ROTACIONAR ESQUERDA");

  motor_1(HORARIO, speed);
  motor_2(HORARIO, speed);
  motor_3(HORARIO, speed);
  motor_4(HORARIO, speed);
}

void move_right(int speed) {
  print_movimento("DIREITA");

  motor_1(ANTI_HORARIO, speed);
  motor_2(HORARIO, speed);
  motor_3(HORARIO, speed);
  motor_4(ANTI_HORARIO, speed);
}

void move_left(int speed) {
  print_movimento("ESQUERDA");

  motor_1(HORARIO, speed);
  motor_2(ANTI_HORARIO, speed);
  motor_3(ANTI_HORARIO, speed);
  motor_4(HORARIO, speed);
}

void stop_robot() {
  print_movimento("PARADO");

  motor_1(PARADO, 0);
  motor_2(PARADO, 0);
  motor_3(PARADO, 0);
  motor_4(PARADO, 0);
}
