#include <Wire.h>
#include <HTInfraredSeeker.h>

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

// PWM:
#define pwm_1 2 // Motor_1
#define pwm_2 3 // Motor_2
#define pwm_3 4 // Motor_3
#define pwm_4 5 // Motor_4

// Fim de curso
#define fim_curso_pin 7

// Se o fim de curso estiver ligado entre o pino e GND, deixe true.
// Se estiver ligado entre o pino e 5V, troque para false e mude o pinMode se necessario.
const bool FIM_CURSO_ATIVO_LOW = true;
const unsigned long DEBOUNCE_FIM_CURSO_MS = 35;

// Sensor ultrassonico
#define trig_ultrassonico 30
#define echo_ultrassonico 31

// LDR / Multiplexador
#define S0 34
#define S1 35
#define S2 36
#define S3 37
#define SIG A0
#define EN 26

int ldr[16];
bool linhaBrancaDetectada = false;
int sensorBranco = -1;

// Antes estava 600. Baixei para detectar branco com leitura menor.
const int LIMIAR_BRANCO = 500;

// true: branco tem leitura maior que preto
// false: branco tem leitura menor que preto
const bool BRANCO_VALOR_MAIOR = true;

#define led_pin 6

const int DISTANCIA_PARAR_CM = 25;

// Protecao contra parede muito perto
const int DISTANCIA_PAREDE_PERTO_CM = 12;
const unsigned long TEMPO_RE_PAREDE_MS = 350;

bool habilitar_debug = false;

// Variavel para saber se o robo ja pegou a bola
bool bolaCapturada = false;

// Direction of the ball
int ballDirecao = 0;

// Intensity of the ball
int ballIntens = 0;

// Filtro para confirmar a direcao da bola
int ultimaDirecaoLida = -1;
int contadorDirecao = 0;
int direcaoConfirmada = 0;

const int LEITURAS_PARA_CONFIRMAR = 3;

// Center of the robot
const int BALL_CENTER = 5;

// Velocidades por motor para cada movimento

// Frente
const int M1_FORWARD_SPEED = 100;
const int M2_FORWARD_SPEED = 100;
const int M3_FORWARD_SPEED = 100;
const int M4_FORWARD_SPEED = 100;

// Re
const int M1_BACK_SPEED = 100;
const int M2_BACK_SPEED = 100;
const int M3_BACK_SPEED = 100;
const int M4_BACK_SPEED = 100;

// Giro para direita
const int M1_RIGHT_ROT_SPEED = 70;
const int M2_RIGHT_ROT_SPEED = 70;
const int M3_RIGHT_ROT_SPEED = 70;
const int M4_RIGHT_ROT_SPEED = 70;

// Giro para esquerda
const int M1_LEFT_ROT_SPEED = 70;
const int M2_LEFT_ROT_SPEED = 70;
const int M3_LEFT_ROT_SPEED = 70;
const int M4_LEFT_ROT_SPEED = 70;

// Movimento lateral direita
const int M1_RIGHT_SPEED = 120;
const int M2_RIGHT_SPEED = 120;
const int M3_RIGHT_SPEED = 120;
const int M4_RIGHT_SPEED = 120;

// Movimento lateral esquerda
const int M1_LEFT_SPEED = 120;
const int M2_LEFT_SPEED = 120;
const int M3_LEFT_SPEED = 120;
const int M4_LEFT_SPEED = 120;

const int M1_SEARCH_SPEED = 65;
const int M2_SEARCH_SPEED = 65;
const int M3_SEARCH_SPEED = 65;
const int M4_SEARCH_SPEED = 65;

// Motor direction
const int HORARIO = 1;
const int ANTI_HORARIO = -1;
const int PARADO = 0;

// MPU6050
const int MPU_ADDR = 0x68;

float gyroZOffset = 0;
float anguloRobo = 0;
unsigned long ultimoTempoGyro = 0;

const int TOLERANCIA_NORTE = 5;
const int NORTH_ROT_SPEED = 150;

// Se o robo girar para o lado errado procurando o norte, troque para true
bool GYRO_INVERTIDO = false;

// Interface serial
int telaSerial = 0;
bool menuImpresso = false;
unsigned long ultimoPrintSerial = 0;
const unsigned long INTERVALO_SERIAL = 1000;

String estadoRobo = "Iniciando";
float ultimaDistanciaCm = 999;

void mpu6050_iniciar();
int16_t ler_gyro_z_raw();
void calibrar_gyro();
void atualizar_gyro();
float normalizar_angulo(float angulo);
float erro_para_norte();

void ir_reader();
bool fim_curso_acionado();
bool leitura_fim_curso_bruta_acionada();
void seguir_bola_simples();

void motor_1(int sentido, int speed);
void motor_2(int sentido, int speed);
void motor_3(int sentido, int speed);
void motor_4(int sentido, int speed);

void move_foward();
void move_back();
void right_rotation();
void left_rotation();
void move_right();
void move_left();
void search_ball();
void stop_robot();

float ler_distancia_cm();
bool parede_muito_perto();
void afastar_da_parede();
void olhar_para_norte();
void andar_ate_distancia();

void imprimir_menu_serial();
void atualizar_serial();
void imprimir_situacao_robo();
void imprimir_sensor_ir();
void imprimir_ultrassonico();
void imprimir_mpu6050();
void imprimir_ldr();

void setMuxChannel(int canal);
void lerLDR();
bool detectarBranco();

void debug_msg(String msg);

void setup() {
  Serial.begin(115200);

  pinMode(in_1_ph_1, OUTPUT);
  pinMode(in_2_ph_1, OUTPUT);
  pinMode(in_3_ph_1, OUTPUT);
  pinMode(in_4_ph_1, OUTPUT);

  pinMode(in_1_ph_2, OUTPUT);
  pinMode(in_2_ph_2, OUTPUT);
  pinMode(in_3_ph_2, OUTPUT);
  pinMode(in_4_ph_2, OUTPUT);

  pinMode(pwm_1, OUTPUT);
  pinMode(pwm_2, OUTPUT);
  pinMode(pwm_3, OUTPUT);
  pinMode(pwm_4, OUTPUT);

  pinMode(fim_curso_pin, INPUT_PULLUP);

  pinMode(trig_ultrassonico, OUTPUT);
  pinMode(echo_ultrassonico, INPUT);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(SIG, INPUT);
  pinMode(EN, OUTPUT);

  digitalWrite(EN, LOW);

  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);

  Wire.begin();

  InfraredSeeker::Initialize();

  mpu6050_iniciar();
  calibrar_gyro();
  anguloRobo = 0;
  ultimoTempoGyro = millis();

  debug_msg("Robo Soccer Iniciado!");

  estadoRobo = "Procurando bola";
  imprimir_menu_serial();
}

void loop() {
  atualizar_serial();

  atualizar_gyro();
  ir_reader();

  // ===== LDR =====
  lerLDR();
  linhaBrancaDetectada = detectarBranco();

  if (linhaBrancaDetectada) {
    stop_robot();
    estadoRobo = "Linha branca detectada!";
    digitalWrite(led_pin, HIGH);
    return;
  } else {
    digitalWrite(led_pin, LOW);
  }

  // ===== PAREDE MUITO PERTO =====
  if (!bolaCapturada && parede_muito_perto()) {
    afastar_da_parede();
    return;
  }

  // ===== FIM DE CURSO =====
  if (fim_curso_acionado() && !bolaCapturada) {
    bolaCapturada = true;

    estadoRobo = "Com a bola";
    stop_robot();
    delay(100);

    estadoRobo = "Voltando para o norte";
    olhar_para_norte();

    stop_robot();
    delay(100);

    estadoRobo = "Indo para o gol";
    andar_ate_distancia();

    estadoRobo = "Parado no gol";
    stop_robot();
    return;
  }

  // ===== JA PEGOU A BOLA =====
  if (bolaCapturada) {
    estadoRobo = "Parado";
    stop_robot();
    return;
  }

  // ===== MOVIMENTO NORMAL =====
  seguir_bola_simples();

  delay(60);
}

void ir_reader() {
  InfraredResult InfraredBall = InfraredSeeker::ReadAC();

  ballDirecao = InfraredBall.Direction;
  ballIntens = InfraredBall.Strength;

  if (habilitar_debug) {
    Serial.print("Direcao: ");
    Serial.print(ballDirecao);
    Serial.print("\tIntensidade: ");
    Serial.print(ballIntens);
    Serial.print("\tFim de curso raw: ");
    Serial.print(digitalRead(fim_curso_pin));
    Serial.print("\tFim de curso filtrado: ");
    Serial.println(fim_curso_acionado() ? "ACIONADO" : "SOLTO");
  }
}

bool leitura_fim_curso_bruta_acionada() {
  int leitura = digitalRead(fim_curso_pin);

  if (FIM_CURSO_ATIVO_LOW) {
    return leitura == LOW;
  } else {
    return leitura == HIGH;
  }
}

bool fim_curso_acionado() {
  static bool ultimoEstadoBruto = false;
  static bool estadoFiltrado = false;
  static unsigned long momentoMudanca = 0;

  bool estadoBruto = leitura_fim_curso_bruta_acionada();

  if (estadoBruto != ultimoEstadoBruto) {
    ultimoEstadoBruto = estadoBruto;
    momentoMudanca = millis();
  }

  if (millis() - momentoMudanca >= DEBOUNCE_FIM_CURSO_MS) {
    estadoFiltrado = estadoBruto;
  }

  return estadoFiltrado;
}

void seguir_bola_simples() {
  if (ballDirecao == 0) {
    estadoRobo = "Procurando bola";
    right_rotation();
    return;
  }

  if (ballDirecao > 3 && ballDirecao < 7) {
    estadoRobo = "Indo ate a bola";
    move_foward();
    return;
  }

  if (ballDirecao < 4) {
    estadoRobo = "Bola a esquerda";
    left_rotation();
    return;
  }

  if (ballDirecao > 6) {
    estadoRobo = "Bola a direita";
    right_rotation();
    return;
  }

  stop_robot();
}

void debug_msg(String msg) {
  if (habilitar_debug) {
    Serial.println(msg);
  }
}

// Motor_1 == Front Left
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

// Motor_2 == Back Left
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

// Motor_3 == Back Right
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

// Motor_4 == Front Right
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

void mpu6050_iniciar() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(100);
}

int16_t ler_gyro_z_raw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 2, true);

  int16_t gyroZ = Wire.read() << 8 | Wire.read();
  return gyroZ;
}

void calibrar_gyro() {
  debug_msg("Calibrando MPU6050... deixe o robo parado");

  long soma = 0;
  const int amostras = 1000;

  for (int i = 0; i < amostras; i++) {
    soma += ler_gyro_z_raw();
    delay(2);
  }

  gyroZOffset = soma / (float)amostras;

  debug_msg("Calibracao concluida");
}

void atualizar_gyro() {
  unsigned long agora = millis();

  if (ultimoTempoGyro == 0) {
    ultimoTempoGyro = agora;
    return;
  }

  float dt = (agora - ultimoTempoGyro) / 1000.0;
  ultimoTempoGyro = agora;

  int16_t gyroZRaw = ler_gyro_z_raw();

  float velocidadeZ = (gyroZRaw - gyroZOffset) / 131.0;

  if (GYRO_INVERTIDO) {
    velocidadeZ = -velocidadeZ;
  }

  anguloRobo += velocidadeZ * dt;
  anguloRobo = normalizar_angulo(anguloRobo);
}

float normalizar_angulo(float angulo) {
  while (angulo > 180) {
    angulo -= 360;
  }

  while (angulo < -180) {
    angulo += 360;
  }

  return angulo;
}

float erro_para_norte() {
  return normalizar_angulo(0 - anguloRobo);
}

void olhar_para_norte() {
  unsigned long inicio = millis();

  while (true) {
    atualizar_gyro();

    float erro = erro_para_norte();
    float erroAbs = erro;

    if (erroAbs < 0) {
      erroAbs = -erroAbs;
    }

    if (erroAbs <= TOLERANCIA_NORTE) {
      stop_robot();
      debug_msg("Norte encontrado!");
      delay(300);
      return;
    }

    if (erro > 0) {
      left_rotation();
    } else {
      right_rotation();
    }

    if (millis() - inicio > 6000) {
      stop_robot();
      debug_msg("Tempo limite procurando norte");
      return;
    }

    delay(10);
  }
}

void move_foward() {
  motor_1(ANTI_HORARIO, M1_FORWARD_SPEED);
  motor_2(ANTI_HORARIO, M2_FORWARD_SPEED);
  motor_3(HORARIO, M3_FORWARD_SPEED);
  motor_4(HORARIO, M4_FORWARD_SPEED);
}

void move_back() {
  motor_1(HORARIO, M1_BACK_SPEED);
  motor_2(HORARIO, M2_BACK_SPEED);
  motor_3(ANTI_HORARIO, M3_BACK_SPEED);
  motor_4(ANTI_HORARIO, M4_BACK_SPEED);
}

void right_rotation() {
  motor_1(ANTI_HORARIO, M1_RIGHT_ROT_SPEED);
  motor_2(ANTI_HORARIO, M2_RIGHT_ROT_SPEED);
  motor_3(ANTI_HORARIO, M3_RIGHT_ROT_SPEED);
  motor_4(ANTI_HORARIO, M4_RIGHT_ROT_SPEED);
}

void left_rotation() {
  motor_1(HORARIO, M1_LEFT_ROT_SPEED);
  motor_2(HORARIO, M2_LEFT_ROT_SPEED);
  motor_3(HORARIO, M3_LEFT_ROT_SPEED);
  motor_4(HORARIO, M4_LEFT_ROT_SPEED);
}

void move_right() {
  motor_1(ANTI_HORARIO, M1_RIGHT_SPEED);
  motor_2(HORARIO, M2_RIGHT_SPEED);
  motor_3(HORARIO, M3_RIGHT_SPEED);
  motor_4(ANTI_HORARIO, M4_RIGHT_SPEED);
}

void move_left() {
  motor_1(HORARIO, M1_LEFT_SPEED);
  motor_2(ANTI_HORARIO, M2_LEFT_SPEED);
  motor_3(ANTI_HORARIO, M3_LEFT_SPEED);
  motor_4(HORARIO, M4_LEFT_SPEED);
}

void search_ball() {
  motor_1(ANTI_HORARIO, M1_SEARCH_SPEED);
  motor_2(ANTI_HORARIO, M2_SEARCH_SPEED);
  motor_3(ANTI_HORARIO, M3_SEARCH_SPEED);
  motor_4(ANTI_HORARIO, M4_SEARCH_SPEED);
}

void stop_robot() {
  motor_1(PARADO, 0);
  motor_2(PARADO, 0);
  motor_3(PARADO, 0);
  motor_4(PARADO, 0);
}

float ler_distancia_cm() {
  long duracao;
  float distancia;

  digitalWrite(trig_ultrassonico, LOW);
  delayMicroseconds(2);

  digitalWrite(trig_ultrassonico, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_ultrassonico, LOW);

  duracao = pulseIn(echo_ultrassonico, HIGH, 30000);

  if (duracao == 0) {
    return 0;
  }

  distancia = duracao * 0.034 / 2;

  return distancia;
}

bool parede_muito_perto() {
  float distancia = ler_distancia_cm();
  ultimaDistanciaCm = distancia;

  if (distancia > 0 && distancia <= DISTANCIA_PAREDE_PERTO_CM) {
    return true;
  }

  return false;
}

void afastar_da_parede() {
  estadoRobo = "Parede perto - dando re";
  digitalWrite(led_pin, HIGH);

  move_back();
  delay(TEMPO_RE_PAREDE_MS);

  stop_robot();
  delay(100);

  digitalWrite(led_pin, LOW);
}

void andar_ate_distancia() {
  estadoRobo = "Indo para o gol";
  digitalWrite(led_pin, LOW);

  while (true) {
    atualizar_serial();
    atualizar_gyro();

    float distancia = ler_distancia_cm();
    ultimaDistanciaCm = distancia;

    if (distancia > 0 && distancia <= DISTANCIA_PARAR_CM) {
      stop_robot();
      estadoRobo = "Parado no gol";
      digitalWrite(led_pin, HIGH);
      delay(300);
      return;
    }

    move_foward();

    delay(60);
  }
}

void imprimir_menu_serial() {
  Serial.println();
  Serial.println("+--------------------------------------+");
  Serial.println("|          MENU DO ROBO SOCCER         |");
  Serial.println("+--------------------------------------+");
  Serial.println("| 1 - Situacao atual do robo           |");
  Serial.println("| 2 - Sensor IR da bola                |");
  Serial.println("| 3 - Sensor ultrassonico              |");
  Serial.println("| 4 - MPU6050 / angulo do robo         |");
  Serial.println("| 5 - Sensores LDR                     |");
  Serial.println("| M - Mostrar este menu novamente      |");
  Serial.println("+--------------------------------------+");
  Serial.println("Digite uma opcao:");
  Serial.println();
}

void atualizar_serial() {
  if (Serial.available() > 0) {
    char opcao = Serial.read();

    if (opcao == '1') {
      telaSerial = 1;
      Serial.println("Tela: Situacao atual do robo");
    } else if (opcao == '2') {
      telaSerial = 2;
      Serial.println("Tela: Sensor IR");
    } else if (opcao == '3') {
      telaSerial = 3;
      Serial.println("Tela: Ultrassonico");
    } else if (opcao == '4') {
      telaSerial = 4;
      Serial.println("Tela: MPU6050");
    } else if (opcao == '5') {
      telaSerial = 5;
      Serial.println("Tela: LDR");
    } else if (opcao == 'm' || opcao == 'M') {
      telaSerial = 0;
      imprimir_menu_serial();
    }
  }

  if (millis() - ultimoPrintSerial < INTERVALO_SERIAL) {
    return;
  }

  ultimoPrintSerial = millis();

  if (telaSerial == 1) {
    imprimir_situacao_robo();
  } else if (telaSerial == 2) {
    imprimir_sensor_ir();
  } else if (telaSerial == 3) {
    imprimir_ultrassonico();
  } else if (telaSerial == 4) {
    imprimir_mpu6050();
  } else if (telaSerial == 5) {
    imprimir_ldr();
  }
}

void imprimir_situacao_robo() {
  Serial.println("+-------------------------------+");
  Serial.println("|       SITUACAO DO ROBO        |");
  Serial.println("+-------------------------------+");
  Serial.print("| Estado: ");
  Serial.println(estadoRobo);
  Serial.print("| Fim de curso raw: ");
  Serial.println(digitalRead(fim_curso_pin));
  Serial.print("| Fim de curso filtrado: ");
  Serial.println(fim_curso_acionado() ? "ACIONADO" : "SOLTO");
  Serial.print("| Bola capturada: ");
  Serial.println(bolaCapturada ? "SIM" : "NAO");
  Serial.print("| Linha branca: ");
  Serial.println(linhaBrancaDetectada ? "SIM" : "NAO");
  Serial.print("| Sensor branco: ");
  Serial.println(sensorBranco);
  Serial.print("| Distancia: ");
  Serial.print(ultimaDistanciaCm);
  Serial.println(" cm");
  Serial.println("+-------------------------------+");
}

void imprimir_sensor_ir() {
  Serial.println("+-------------------------------+");
  Serial.println("|          SENSOR IR            |");
  Serial.println("+-------------------------------+");
  Serial.print("| Direcao da bola: ");
  Serial.println(ballDirecao);
  Serial.print("| Intensidade: ");
  Serial.println(ballIntens);
  Serial.println("+-------------------------------+");
}

void imprimir_ultrassonico() {
  ultimaDistanciaCm = ler_distancia_cm();

  Serial.println("+-------------------------------+");
  Serial.println("|       SENSOR ULTRASSONICO     |");
  Serial.println("+-------------------------------+");

  if (ultimaDistanciaCm == 0) {
    Serial.println("| Distancia: sem leitura        |");
  } else {
    Serial.print("| Distancia: ");
    Serial.print(ultimaDistanciaCm);
    Serial.println(" cm");
  }

  Serial.print("| Parar no gol <= ");
  Serial.print(DISTANCIA_PARAR_CM);
  Serial.println(" cm");

  Serial.print("| Dar re na parede <= ");
  Serial.print(DISTANCIA_PAREDE_PERTO_CM);
  Serial.println(" cm");

  Serial.println("+-------------------------------+");
}

void imprimir_mpu6050() {
  Serial.println("+-------------------------------+");
  Serial.println("|           MPU6050             |");
  Serial.println("+-------------------------------+");
  Serial.print("| Angulo atual: ");
  Serial.print(anguloRobo);
  Serial.println(" graus");
  Serial.print("| Erro para norte: ");
  Serial.print(erro_para_norte());
  Serial.println(" graus");
  Serial.println("+-------------------------------+");
}

void imprimir_ldr() {
  Serial.println("+-------------------------------+");
  Serial.println("|             LDR               |");
  Serial.println("+-------------------------------+");

  for (int i = 0; i < 16; i++) {
    Serial.print("| LDR ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(ldr[i]);
  }

  Serial.print("| Linha branca: ");
  Serial.println(linhaBrancaDetectada ? "SIM" : "NAO");

  Serial.print("| Sensor branco: ");
  Serial.println(sensorBranco);

  Serial.print("| Limiar: ");
  Serial.println(LIMIAR_BRANCO);

  Serial.print("| Branco valor maior: ");
  Serial.println(BRANCO_VALOR_MAIOR ? "SIM" : "NAO");

  Serial.println("+-------------------------------+");
}

void setMuxChannel(int canal) {
  digitalWrite(S0, canal & 0x01);
  digitalWrite(S1, (canal >> 1) & 0x01);
  digitalWrite(S2, (canal >> 2) & 0x01);
  digitalWrite(S3, (canal >> 3) & 0x01);
}

void lerLDR() {
  for (int i = 0; i < 16; i++) {
    setMuxChannel(i);

    delayMicroseconds(300);

    analogRead(SIG);
    ldr[i] = analogRead(SIG);
  }
}

bool detectarBranco() {
  sensorBranco = -1;

  for (int i = 0; i < 16; i++) {
    bool brancoDetectado;

    if (BRANCO_VALOR_MAIOR) {
      brancoDetectado = ldr[i] > LIMIAR_BRANCO;
    } else {
      brancoDetectado = ldr[i] < LIMIAR_BRANCO;
    }

    if (brancoDetectado) {
      sensorBranco = i;
      return true;
    }
  }

  return false;
}
