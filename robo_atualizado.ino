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
#define pwm_1 3
#define pwm_4 4
#define pwm_3 5

#define DRIBBLER_ESC_PIN 9
#define DRIBBLER_PARADO_US 1000
#define DRIBBLER_VELOCIDADE_PROCURA_US 1170
#define DRIBBLER_VELOCIDADE_COM_BOLA_US 1300

#define fim_curso_pin 7

#define trig_ultrassonico 30
#define echo_frente 31
#define echo_esquerda 32
#define echo_direita 33

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

int velocidadeFrente = 190;
int velocidadeGiro = 115;
int velocidadeBusca = 105;
int velocidadeReLinha = 190;
int velocidadeGiroComBola = 180;

int ballDirecao = 0;

bool bolaCapturada = false;
bool ignorarFimCursoAteSoltar = false;
bool dribblerLigado = false;
int dribblerMicroAtual = DRIBBLER_PARADO_US;

float gyroZOffset = 0;
float anguloRobo = 0;
unsigned long ultimoTempoGyro = 0;
unsigned long ultimoPrintSerial = 0;

const int TOLERANCIA_ANGULO = 5;
const int ANGULO_SUL = 180;
const int LIMIAR_LINHA = 70; // alterar para sensor de linha na quadra
const bool LINHA_VALOR_MENOR_QUE_QUADRA = true;
const unsigned long TEMPO_ENCAIXAR_BOLA_MS = 1500;
const unsigned long TEMPO_GIRO_COM_BOLA_MS = 800;
const unsigned long TEMPO_RE_LINHA_MS = 450;

bool GYRO_INVERTIDO = false;

int ldr[16];
bool linhaPretaDetectada = false;
int sensorLinhaPreta = -1;

float ultimaDistanciaFrenteCm = 0;
float ultimaDistanciaEsquerdaCm = 0;
float ultimaDistanciaDireitaCm = 0;

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
  pinMode(echo_esquerda, INPUT);
  pinMode(echo_direita, INPUT);

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
  dribbler_parado();
  delay(3000);
  dribbler_velocidade_procura();

  Serial.println("Robo iniciado");
}

void loop() {
  atualizar_gyro();

  if (verificar_linha_preta()) {
    return;
  }

  ir_reader();
  monitor_serial();

  if (ignorarFimCursoAteSoltar && !fim_curso_acionado()) {
    ignorarFimCursoAteSoltar = false;
  }

  if (fim_curso_acionado() && !bolaCapturada && !ignorarFimCursoAteSoltar) {
    bolaCapturada = true;

    stop_robot();
    dribbler_velocidade_procura();
    Serial.println("Bola encostou no fim de curso. Esperando encaixar...");
    if (!esperar_bola_encaixar()) {
      bolaCapturada = false;
      Serial.println("Bola escapou antes de encaixar. Voltando para procurar.");
      return;
    }

    dribbler_velocidade_com_bola();
    delay(100);

    Serial.println("Bola capturada. Virando para o sul...");
    if (!olhar_para_sul_com_bola()) {
      Serial.println("Bola escapou ao alinhar. Voltando para procurar.");
      return;
    }

    stop_robot();
    delay(200);

    Serial.println("Comparando ultrassonicos laterais...");
    if (!girar_pelo_menor_ultrassonico_lateral()) {
      Serial.println("Bola escapou no giro final. Voltando para procurar.");
      return;
    }

    stop_robot();
    dribbler_velocidade_procura();
    bolaCapturada = false;
    ignorarFimCursoAteSoltar = true;

    Serial.println("Finalizado. Voltando para procurar a bolinha.");
    return;
  }

  if (bolaCapturada) {
    dribbler_velocidade_com_bola();
    stop_robot();
    return;
  }

  dribbler_velocidade_procura();
  seguir_bola_simples();

  delay(25);
}

void monitor_serial() {
  if (millis() - ultimoPrintSerial < 500) {
    return;
  }

  ultimoPrintSerial = millis();
  atualizar_ultrassonicos();

  Serial.println("-------------------------------");

  Serial.print("Direcao IR: ");
  Serial.println(ballDirecao);

  Serial.print("Fim de curso: ");
  Serial.println(fim_curso_acionado() ? "ACIONADO" : "SOLTO");

  Serial.print("Angulo MPU: ");
  Serial.println(anguloRobo);

  Serial.print("Erro para Sul: ");
  Serial.println(erro_para_sul());

  Serial.print("Dribbler: ");
  Serial.print(dribblerLigado ? "LIGADO" : "PARADO");
  Serial.print(" / ");
  Serial.print(dribblerMicroAtual);
  Serial.println(" us");

  Serial.print("Linha preta: ");
  Serial.print(linhaPretaDetectada ? "SIM" : "NAO");
  Serial.print(" / Sensor: ");
  Serial.println(sensorLinhaPreta);

  Serial.print("Ultrassonico frente: ");
  Serial.print(ultimaDistanciaFrenteCm);
  Serial.println(" cm");

  Serial.print("Ultrassonico esquerda: ");
  Serial.print(ultimaDistanciaEsquerdaCm);
  Serial.println(" cm");

  Serial.print("Ultrassonico direita: ");
  Serial.print(ultimaDistanciaDireitaCm);
  Serial.println(" cm");

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

bool esperar_bola_encaixar() {
  unsigned long inicio = millis();

  while (millis() - inicio < TEMPO_ENCAIXAR_BOLA_MS) {
    atualizar_gyro();

    if (verificar_linha_preta()) {
      inicio = millis();
      continue;
    }

    if (!fim_curso_acionado()) {
      stop_robot();
      dribbler_velocidade_procura();
      return false;
    }

    stop_robot();
    delay(20);
  }

  return true;
}

bool girar_pelo_menor_ultrassonico_lateral() {
  if (!fim_curso_acionado()) {
    stop_robot();
    dribbler_velocidade_procura();
    return false;
  }

  atualizar_ultrassonicos();
  stop_robot();

  Serial.print("Distancia esquerda: ");
  Serial.println(ultimaDistanciaEsquerdaCm);
  Serial.print("Distancia direita: ");
  Serial.println(ultimaDistanciaDireitaCm);

  if (ultimaDistanciaDireitaCm <= 0 && ultimaDistanciaEsquerdaCm <= 0) {
    Serial.println("Laterais sem leitura. Girando para a esquerda por padrao.");
    return girar_esquerda_com_bola_por_tempo(TEMPO_GIRO_COM_BOLA_MS);
  }

  if (ultimaDistanciaDireitaCm > 0 && (ultimaDistanciaEsquerdaCm <= 0 || ultimaDistanciaDireitaCm < ultimaDistanciaEsquerdaCm)) {
    Serial.println("Direita menor. Girando para a esquerda.");
    return girar_esquerda_com_bola_por_tempo(TEMPO_GIRO_COM_BOLA_MS);
  }

  if (ultimaDistanciaEsquerdaCm > 0 && (ultimaDistanciaDireitaCm <= 0 || ultimaDistanciaEsquerdaCm < ultimaDistanciaDireitaCm)) {
    Serial.println("Esquerda menor. Girando para a direita.");
    return girar_direita_com_bola_por_tempo(TEMPO_GIRO_COM_BOLA_MS);
  }

  Serial.println("Laterais iguais. Girando para a esquerda por padrao.");
  return girar_esquerda_com_bola_por_tempo(TEMPO_GIRO_COM_BOLA_MS);
}

bool girar_esquerda_com_bola_por_tempo(unsigned long tempoMs) {
  return girar_com_bola_por_tempo(true, tempoMs);
}

bool girar_direita_com_bola_por_tempo(unsigned long tempoMs) {
  return girar_com_bola_por_tempo(false, tempoMs);
}

bool girar_com_bola_por_tempo(bool paraEsquerda, unsigned long tempoMs) {
  unsigned long inicio = millis();

  dribbler_velocidade_com_bola();

  while (millis() - inicio < tempoMs) {
    atualizar_gyro();

    if (verificar_linha_preta()) {
      inicio = millis();
      continue;
    }

    if (!fim_curso_acionado()) {
      stop_robot();
      dribbler_velocidade_procura();
      bolaCapturada = false;
      return false;
    }

    if (paraEsquerda) {
      left_rotation(velocidadeGiroComBola);
    } else {
      right_rotation(velocidadeGiroComBola);
    }

    delay(10);
  }

  stop_robot();
  return true;
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
    dribbler_velocidade_com_bola();
  } else {
    dribbler_velocidade_procura();
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

void atualizar_ultrassonicos() {
  ultimaDistanciaFrenteCm = ler_distancia_cm(echo_frente);
  delay(20);

  ultimaDistanciaEsquerdaCm = ler_distancia_cm(echo_esquerda);
  delay(20);

  ultimaDistanciaDireitaCm = ler_distancia_cm(echo_direita);
  delay(20);
}

float ler_distancia_cm(int echoPin) {
  digitalWrite(trig_ultrassonico, LOW);
  delayMicroseconds(2);

  digitalWrite(trig_ultrassonico, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_ultrassonico, LOW);

  long duracao = pulseIn(echoPin, HIGH, 30000);

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

float erro_para_sul() {
  return normalizar_angulo(ANGULO_SUL - anguloRobo);
}

void olhar_para_sul() {
  girar_ate_angulo(ANGULO_SUL, velocidadeGiro, 6000);
}

bool olhar_para_sul_com_bola() {
  return girar_ate_angulo_com_bola(ANGULO_SUL, velocidadeGiro, 6000);
}

void girar_ate_angulo(float alvo, int velocidade, unsigned long tempoMaximo) {
  unsigned long inicio = millis();

  while (true) {
    atualizar_gyro();

    if (verificar_linha_preta()) {
      continue;
    }

    monitor_serial();

    float erro = normalizar_angulo(alvo - anguloRobo);
    float erroAbs = abs(erro);

    if (erroAbs <= TOLERANCIA_ANGULO) {
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

bool girar_ate_angulo_com_bola(float alvo, int velocidade, unsigned long tempoMaximo) {
  unsigned long inicio = millis();

  while (true) {
    atualizar_gyro();

    if (verificar_linha_preta()) {
      continue;
    }

    if (!fim_curso_acionado()) {
      stop_robot();
      dribbler_velocidade_procura();
      bolaCapturada = false;
      return false;
    }

    monitor_serial();

    float erro = normalizar_angulo(alvo - anguloRobo);
    float erroAbs = abs(erro);

    if (erroAbs <= TOLERANCIA_ANGULO) {
      stop_robot();
      return true;
    }

    if (erro > 0) {
      left_rotation(velocidade);
    } else {
      right_rotation(velocidade);
    }

    if (millis() - inicio > tempoMaximo) {
      stop_robot();
      return true;
    }

    delay(10);
  }
}

void dribbler_velocidade_procura() {
  esc.writeMicroseconds(DRIBBLER_VELOCIDADE_PROCURA_US);
  dribblerMicroAtual = DRIBBLER_VELOCIDADE_PROCURA_US;
  dribblerLigado = true;
}

void dribbler_parado() {
  esc.writeMicroseconds(DRIBBLER_PARADO_US);
  dribblerMicroAtual = DRIBBLER_PARADO_US;
  dribblerLigado = false;
}

void dribbler_velocidade_com_bola() {
  esc.writeMicroseconds(DRIBBLER_VELOCIDADE_COM_BOLA_US);
  dribblerMicroAtual = DRIBBLER_VELOCIDADE_COM_BOLA_US;
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

void move_right(int v) {
  motor_1(ANTI_HORARIO, v);
  motor_2(HORARIO, v);
  motor_3(HORARIO, v);
  motor_4(HORARIO, v);
}

void move_left(int v) {
  motor_1(HORARIO, v);
  motor_2(ANTI_HORARIO, v);
  motor_3(ANTI_HORARIO, v);
  motor_4(ANTI_HORARIO, v);
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


