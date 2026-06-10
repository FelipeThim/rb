#include <Wire.h>
#include <Servo.h>
#include <HTInfraredSeeker.h>

#define in_1_ph_1 25
#define in_2_ph_1 24
#define in_3_ph_1 23
#define in_4_ph_1 22

#define in_1_ph_2 47
#define in_2_ph_2 46
#define in_3_ph_2 49
#define in_4_ph_2 48

#define pwm_2 2
#define pwm_1 1
#define pwm_4 4
#define pwm_3 5

#define DRIBBLER_ESC_PIN 9
#define DRIBBLER_MIN_US 1000
#define DRIBBLER_LENTO_US 1100
#define DRIBBLER_PARTIDA_US 1200
#define DRIBBLER_FORTE_US 1250

#define fim_curso_pin 7

#define trig_ultrassonico 30
#define echo_frente 31

#define S0 34
#define S1 35
#define S2 36
#define S3 37
#define SIG A0
#define EN 26

#define led_pin 6

#define HORARIO 1
#define ANTI_HORARIO -1
#define PARADO 0

#define MPU_ADDR 0x68

Servo esc;

int velocidadeFrente = 170;
int velocidadeGiro = 95;
int velocidadeBusca = 85;
int velocidadeReLinha = 130;

int ballDirecao = 0;

bool bolaCapturada = false;
bool ignorarFimCursoAteSoltar = false;
bool dribblerLigado = false;
int dribblerMicroAtual = DRIBBLER_MIN_US;

float gyroZOffset = 0;
float anguloRobo = 0;
unsigned long ultimoTempoGyro = 0;
unsigned long ultimoPrintSerial = 0;

const int TOLERANCIA_NORTE = 5;
const int DISTANCIA_PARAR_CM = 25;
const int LIMIAR_LINHA = 500;
const bool LINHA_VALOR_MENOR_QUE_QUADRA = true;
const unsigned long TEMPO_RE_LINHA_MS = 450;

bool GYRO_INVERTIDO = false;

int ldr[16];
bool linhaPretaDetectada = false;
int sensorLinhaPreta = -1;

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
  pinMode(echo_frente, INPUT);

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

  esc.attach(DRIBBLER_ESC_PIN);
  esc.writeMicroseconds(DRIBBLER_MIN_US);
  delay(3000);
  dribbler_iniciar_ligado();

  Serial.println("Robo iniciado");
}

void loop() {
  atualizar_gyro();
  ir_reader();
  monitor_serial();

  if (verificar_linha_preta()) {
    return;
  }

  if (ignorarFimCursoAteSoltar && !fim_curso_acionado()) {
    ignorarFimCursoAteSoltar = false;
  }

  if (fim_curso_acionado() && !bolaCapturada && !ignorarFimCursoAteSoltar) {
    bolaCapturada = true;

    stop_robot();
    dribbler_forte();
    delay(200);

    Serial.println("Bola capturada. Voltando para o norte...");
    olhar_para_norte();

    stop_robot();
    delay(300);

    Serial.println("Indo para o gol...");
    andar_ate_gol();

    stop_robot();
    dribbler_lento();
    bolaCapturada = false;
    ignorarFimCursoAteSoltar = true;

    Serial.println("Finalizado. Voltando para procurar a bolinha.");
    return;
  }

  if (bolaCapturada) {
    dribbler_forte();
    stop_robot();
    return;
  }

  dribbler_lento();
  seguir_bola_simples();

  delay(25);
}

void monitor_serial() {
  if (millis() - ultimoPrintSerial < 500) {
    return;
  }

  ultimoPrintSerial = millis();

  Serial.println("-------------------------------");

  Serial.print("Direcao IR: ");
  Serial.println(ballDirecao);

  Serial.print("Fim de curso: ");
  Serial.println(fim_curso_acionado() ? "ACIONADO" : "SOLTO");

  Serial.print("Angulo MPU: ");
  Serial.println(anguloRobo);

  Serial.print("Erro para Norte: ");
  Serial.println(erro_para_norte());

  Serial.print("Dribbler: ");
  Serial.print(dribblerLigado ? "LIGADO" : "PARADO");
  Serial.print(" / ");
  Serial.print(dribblerMicroAtual);
  Serial.println(" us");

  Serial.print("Linha preta: ");
  Serial.print(linhaPretaDetectada ? "SIM" : "NAO");
  Serial.print(" / Sensor: ");
  Serial.println(sensorLinhaPreta);

  Serial.println("-------------------------------");
}

void ir_reader() {
  InfraredResult bola = InfraredSeeker::ReadAC();

  ballDirecao = bola.Direction;
}

void seguir_bola_simples() {
  if (ballDirecao == 0) {
    right_rotation(velocidadeBusca);
    return;
  }

  if (ballDirecao == 4 || ballDirecao == 5 || ballDirecao == 6) {
    move_foward(velocidadeFrente);
    return;
  }

  if (ballDirecao == 1 || ballDirecao == 2 || ballDirecao == 3) {
    left_rotation(velocidadeGiro);
    return;
  }

  if (ballDirecao == 7 || ballDirecao == 8 || ballDirecao == 9) {
    right_rotation(velocidadeGiro);
    return;
  }

  stop_robot();
}

bool fim_curso_acionado() {
  return digitalRead(fim_curso_pin) == LOW;
}

void andar_ate_gol() {
  while (true) {
    atualizar_gyro();
    monitor_serial();

    if (verificar_linha_preta()) {
      continue;
    }

    float distancia = ler_distancia_cm();

    Serial.print("Distancia frente: ");
    Serial.println(distancia);

    if (distancia > 0 && distancia <= DISTANCIA_PARAR_CM) {
      stop_robot();
      delay(200);

      Serial.println("Gol a 25 cm. Parando.");

      return;
    }

    dribbler_forte();
    move_foward(velocidadeFrente);

    delay(50);
  }
}

bool verificar_linha_preta() {
  lerLDR();
  linhaPretaDetectada = detectar_linha_preta();

  if (!linhaPretaDetectada) {
    digitalWrite(led_pin, LOW);
    return false;
  }

  digitalWrite(led_pin, HIGH);
  Serial.print("Linha preta detectada no sensor ");
  Serial.println(sensorLinhaPreta);

  afastar_da_linha_preta();
  return true;
}

void afastar_da_linha_preta() {
  if (bolaCapturada) {
    dribbler_forte();
  } else {
    dribbler_lento();
  }

  move_back(velocidadeReLinha);
  delay(TEMPO_RE_LINHA_MS);
  stop_robot();
  delay(100);
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

bool detectar_linha_preta() {
  sensorLinhaPreta = -1;

  for (int i = 0; i < 16; i++) {
    bool linhaDetectada;

    if (LINHA_VALOR_MENOR_QUE_QUADRA) {
      linhaDetectada = ldr[i] < LIMIAR_LINHA;
    } else {
      linhaDetectada = ldr[i] > LIMIAR_LINHA;
    }

    if (linhaDetectada) {
      sensorLinhaPreta = i;
      return true;
    }
  }

  return false;
}

float ler_distancia_cm() {
  digitalWrite(trig_ultrassonico, LOW);
  delayMicroseconds(2);

  digitalWrite(trig_ultrassonico, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_ultrassonico, LOW);

  long duracao = pulseIn(echo_frente, HIGH, 30000);

  if (duracao == 0) {
    return 0;
  }

  return duracao * 0.034 / 2;
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
  girar_ate_angulo(0, velocidadeGiro, 6000);
}

void girar_ate_angulo(float alvo, int velocidade, unsigned long tempoMaximo) {
  unsigned long inicio = millis();

  while (true) {
    atualizar_gyro();
    monitor_serial();

    float erro = normalizar_angulo(alvo - anguloRobo);
    float erroAbs = abs(erro);

    if (erroAbs <= TOLERANCIA_NORTE) {
      stop_robot();
      return;
    }

    if (erro > 0) {
      left_rotation(velocidade);
    } else {
      right_rotation(velocidade);
    }

    if (millis() - inicio > tempoMaximo) {
      stop_robot();
      return;
    }

    delay(10);
  }
}

void dribbler_lento() {
  esc.writeMicroseconds(DRIBBLER_LENTO_US);
  dribblerMicroAtual = DRIBBLER_LENTO_US;
  dribblerLigado = true;
}

void dribbler_iniciar_ligado() {
  esc.writeMicroseconds(DRIBBLER_PARTIDA_US);
  dribblerMicroAtual = DRIBBLER_PARTIDA_US;
  dribblerLigado = true;
  delay(600);
  dribbler_lento();
}

void dribbler_forte() {
  esc.writeMicroseconds(DRIBBLER_FORTE_US);
  dribblerMicroAtual = DRIBBLER_FORTE_US;
  dribblerLigado = true;
}

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

void move_foward(int v) {
  motor_1(ANTI_HORARIO, v);
  motor_2(ANTI_HORARIO, v);
  motor_3(HORARIO, v);
  motor_4(ANTI_HORARIO, v);
}

void move_back(int v) {
  motor_1(HORARIO, v);
  motor_2(HORARIO, v);
  motor_3(ANTI_HORARIO, v);
  motor_4(HORARIO, v);
}

void left_rotation(int v) {
  motor_1(ANTI_HORARIO, v);
  motor_2(HORARIO, v);
  motor_3(ANTI_HORARIO, v);
  motor_4(ANTI_HORARIO, v);
}

void right_rotation(int v) {
  motor_1(HORARIO, v);
  motor_2(ANTI_HORARIO, v);
  motor_3(HORARIO, v);
  motor_4(HORARIO, v);
}

void stop_robot() {
  motor_1(PARADO, 0);
  motor_2(PARADO, 0);
  motor_3(PARADO, 0);
  motor_4(PARADO, 0);
}
