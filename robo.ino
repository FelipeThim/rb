#include <Wire.h>
#include <Servo.h>
#include <HTInfraredSeeker.h>

// Ponte H 1
#define in_1_ph_1 25 // Motor 2
#define in_2_ph_1 24 // Motor 2
#define in_3_ph_1 23 // Motor 1
#define in_4_ph_1 22 // Motor 1

// Ponte H 2
#define in_1_ph_2 47 // Motor 4
#define in_2_ph_2 46 // Motor 4
#define in_3_ph_2 49 // Motor 3
#define in_4_ph_2 48 // Motor 3

// PWM calibrado
#define pwm_2 2 // Motor 2
#define pwm_1 3 // Motor 1
#define pwm_4 4 // Motor 4
#define pwm_3 5 // Motor 3

// Fim de curso
#define fim_curso_pin 7

// Dribbler / ESC
#define DRIBBLER_ESC_PIN 9
#define DRIBBLER_MIN_US 1000
#define DRIBBLER_MAX_US 2000

Servo esc;
int dribblerMicroAtual = DRIBBLER_MIN_US;
bool dribblerLigado = false;

// Fim de curso
const bool FIM_CURSO_ATIVO_LOW = true;
const unsigned long DEBOUNCE_FIM_CURSO_MS = 35;

// Ultrassonico
#define trig_ultrassonico 30
#define echo_ultrassonico 31

// LDR / Multiplexador
#define S0 34
#define S1 35
#define S2 36
#define S3 37
#define SIG A0
#define EN 26

#define led_pin 6

int ldr[16];
bool linhaBrancaDetectada = false;
int sensorBranco = -1;

const int LIMIAR_BRANCO = 500;
const bool BRANCO_VALOR_MAIOR = true;

const int DISTANCIA_PARAR_CM = 25;
const int DISTANCIA_PAREDE_PERTO_CM = 12;
const unsigned long TEMPO_RE_PAREDE_MS = 350;

bool habilitar_debug = false;
bool bolaCapturada = false;

int ballDirecao = 0;
int ballIntens = 0;

// Velocidades calibradas
const int M1_FORWARD_SPEED = 220;
const int M2_FORWARD_SPEED = 255;
const int M3_FORWARD_SPEED = 140;
const int M4_FORWARD_SPEED = 255;

const int M1_BACK_SPEED = 220;
const int M2_BACK_SPEED = 255;
const int M3_BACK_SPEED = 140;
const int M4_BACK_SPEED = 255;

const int M1_LEFT_ROT_SPEED = 220;
const int M2_LEFT_ROT_SPEED = 255;
const int M3_LEFT_ROT_SPEED = 140;
const int M4_LEFT_ROT_SPEED = 255;

const int M1_RIGHT_ROT_SPEED = 220;
const int M2_RIGHT_ROT_SPEED = 255;
const int M3_RIGHT_ROT_SPEED = 140;
const int M4_RIGHT_ROT_SPEED = 255;

const int HORARIO = 1;
const int ANTI_HORARIO = -1;
const int PARADO = 0;

// MPU6050
const int MPU_ADDR = 0x68;

float gyroZOffset = 0;
float anguloRobo = 0;
unsigned long ultimoTempoGyro = 0;

const int TOLERANCIA_NORTE = 5;
bool GYRO_INVERTIDO = false;

// Serial
int telaSerial = 0;
unsigned long ultimoPrintSerial = 0;
const unsigned long INTERVALO_SERIAL = 1000;

String estadoRobo = "Iniciando";
float ultimaDistanciaCm = 999;

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

  dribbler_iniciar();

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

  stop_robot();

  estadoRobo = "Procurando bola";
  imprimir_menu_serial();

  Serial.println("Robo Soccer iniciado com motores calibrados");
}

void loop() {
  atualizar_serial();

  atualizar_gyro();
  ir_reader();

  lerLDR();
  linhaBrancaDetectada = detectarBranco();

  if (linhaBrancaDetectada) {
    stop_robot();
    dribbler_parar();
    estadoRobo = "Linha branca detectada!";
    digitalWrite(led_pin, HIGH);
    return;
  } else {
    digitalWrite(led_pin, LOW);
  }

  if (!bolaCapturada && parede_muito_perto()) {
    dribbler_parar();
    afastar_da_parede();
    return;
  }

  if (fim_curso_acionado() && !bolaCapturada) {
    bolaCapturada = true;

    estadoRobo = "Com a bola";
    stop_robot();
    dribbler_segurar();
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

  if (bolaCapturada) {
    estadoRobo = "Parado";
    stop_robot();
    dribbler_parar();
    return;
  }

  dribbler_segurar();
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
    Serial.print(" | Intensidade: ");
    Serial.println(ballIntens);
  }
}

void seguir_bola_simples() {
  if (ballDirecao == 0) {
    estadoRobo = "Procurando bola";
    right_rotation();
    return;
  }

  if (ballDirecao == 5) {
    estadoRobo = "Indo ate a bola";
    move_foward();
    return;
  }

  if (ballDirecao == 1 || ballDirecao == 2 || ballDirecao == 3 || ballDirecao == 4) {
    estadoRobo = "Bola a esquerda";
    left_rotation();
    return;
  }

  if (ballDirecao == 6 || ballDirecao == 7 || ballDirecao == 8 || ballDirecao == 9) {
    estadoRobo = "Bola a direita";
    right_rotation();
    return;
  }

  stop_robot();
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

// ===== DRIBBLER / ESC NOVO =====

void dribbler_iniciar() {
  esc.attach(DRIBBLER_ESC_PIN);
  esc.writeMicroseconds(DRIBBLER_MIN_US);
  dribblerMicroAtual = DRIBBLER_MIN_US;
  dribblerLigado = false;
  delay(3000);
}

void dribbler_segurar() {
  esc.writeMicroseconds(DRIBBLER_MAX_US);
  dribblerMicroAtual = DRIBBLER_MAX_US;
  dribblerLigado = true;
}

void dribbler_chutar() {
  for (int velocidade = DRIBBLER_MIN_US; velocidade <= DRIBBLER_MAX_US; velocidade += 10) {
    esc.writeMicroseconds(velocidade);
    dribblerMicroAtual = velocidade;
    delay(50);
  }

  delay(3000);

  for (int velocidade = DRIBBLER_MAX_US; velocidade >= DRIBBLER_MIN_US; velocidade -= 10) {
    esc.writeMicroseconds(velocidade);
    dribblerMicroAtual = velocidade;
    delay(50);
  }

  dribblerLigado = false;
}

void dribbler_parar() {
  esc.writeMicroseconds(DRIBBLER_MIN_US);
  dribblerMicroAtual = DRIBBLER_MIN_US;
  dribblerLigado = false;
}

// ===== MOTORES =====

void motor_1(int sentido, int speed) {
  if (sentido == HORARIO) {
    digitalWrite(in_3_ph_1, HIGH);
    digitalWrite(in_4_ph_1, LOW);
  } else if (sentido == ANTI_HORARIO) {
    digitalWrite(in_3_ph_1, LOW);
    digitalWrite(in_4_ph_1, HIGH);
  } else {
    digitalWrite(in_3_ph_1, LOW);
    digitalWrite(in_4_ph_1, LOW);
  }

  analogWrite(pwm_1, speed);
}

void motor_2(int sentido, int speed) {
  if (sentido == HORARIO) {
    digitalWrite(in_1_ph_1, HIGH);
    digitalWrite(in_2_ph_1, LOW);
  } else if (sentido == ANTI_HORARIO) {
    digitalWrite(in_1_ph_1, LOW);
    digitalWrite(in_2_ph_1, HIGH);
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

void move_foward() {
  motor_1(ANTI_HORARIO, M1_FORWARD_SPEED);
  motor_2(ANTI_HORARIO, M2_FORWARD_SPEED);
  motor_3(HORARIO, M3_FORWARD_SPEED);
  motor_4(ANTI_HORARIO, M4_FORWARD_SPEED);
}

void move_back() {
  motor_1(HORARIO, M1_BACK_SPEED);
  motor_2(HORARIO, M2_BACK_SPEED);
  motor_3(ANTI_HORARIO, M3_BACK_SPEED);
  motor_4(HORARIO, M4_BACK_SPEED);
}

void left_rotation() {
  motor_1(ANTI_HORARIO, M1_LEFT_ROT_SPEED);
  motor_2(HORARIO, M2_LEFT_ROT_SPEED);
  motor_3(ANTI_HORARIO, M3_LEFT_ROT_SPEED);
  motor_4(ANTI_HORARIO, M4_LEFT_ROT_SPEED);
}

void right_rotation() {
  motor_1(HORARIO, M1_RIGHT_ROT_SPEED);
  motor_2(ANTI_HORARIO, M2_RIGHT_ROT_SPEED);
  motor_3(HORARIO, M3_RIGHT_ROT_SPEED);
  motor_4(HORARIO, M4_RIGHT_ROT_SPEED);
}

void stop_robot() {
  motor_1(PARADO, 0);
  motor_2(PARADO, 0);
  motor_3(PARADO, 0);
  motor_4(PARADO, 0);
}

// ===== MPU6050 =====

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
  Serial.println("Calibrando MPU6050... deixe o robo parado");

  long soma = 0;
  const int amostras = 1000;

  for (int i = 0; i < amostras; i++) {
    soma += ler_gyro_z_raw();
    delay(2);
  }

  gyroZOffset = soma / (float)amostras;

  Serial.println("Calibracao concluida");
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
      return;
    }

    delay(10);
  }
}

// ===== ULTRASSONICO =====

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
  dribbler_segurar();

  while (true) {
    atualizar_serial();
    atualizar_gyro();

    float distancia = ler_distancia_cm();
    ultimaDistanciaCm = distancia;

    if (distancia > 0 && distancia <= DISTANCIA_PARAR_CM) {
      stop_robot();
      estadoRobo = "Chutando";
      digitalWrite(led_pin, HIGH);
      dribbler_chutar();
      estadoRobo = "Parado no gol";
      delay(300);
      return;
    }

    dribbler_segurar();
    move_foward();

    delay(60);
  }
}

// ===== LDR =====

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

// ===== SERIAL =====

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
  Serial.print("| Dribbler: ");
  Serial.print(dribblerLigado ? "LIGADO" : "PARADO");
  Serial.print(" / ");
  Serial.print(dribblerMicroAtual);
  Serial.println(" us");
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
