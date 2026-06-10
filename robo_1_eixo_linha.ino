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
#define DRIBBLER_MIN_US 1000
#define DRIBBLER_PROCURA_US 1170
#define DRIBBLER_COM_BOLA_US 1300
#define DRIBBLER_CHUTE_US 1350

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

int velocidadeFrente = 175;
int velocidadeGiro = 115;
int velocidadeBusca = 105;
int velocidadeReLinha = 175;
int velocidadeGiroChute = 175;
int velocidadeEixo = 135;
int velocidadeCorrecaoLinha = 95;
int velocidadeBuscaLinha = 80;

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
const int ANGULO_SUL = 180;
const int LIMIAR_LINHA = 70; // alterar para sensor de linha na quadra
const bool LINHA_VALOR_MENOR_QUE_QUADRA = true;
const int SENSOR_LINHA_ESQUERDA_FIM = 5;
const int SENSOR_LINHA_CENTRO_INICIO = 6;
const int SENSOR_LINHA_CENTRO_FIM = 9;
const int SENSOR_LINHA_DIREITA_INICIO = 10;
const unsigned long TEMPO_BUSCA_LINHA_MS = 250;
const unsigned long TEMPO_ENCAIXAR_BOLA_MS = 1200;
const unsigned long TEMPO_GIRO_CHUTE_MS = 700;
const unsigned long TEMPO_RE_LINHA_MS = 700;

bool GYRO_INVERTIDO = false;

int ldr[16];
bool linhaPretaDetectada = false;
int sensorLinhaPreta = -1;

enum PosicaoLinha {
  LINHA_PERDIDA,
  LINHA_ESQUERDA,
  LINHA_CENTRO,
  LINHA_DIREITA
};

PosicaoLinha posicaoLinha = LINHA_PERDIDA;
unsigned long ultimoTempoLinhaVista = 0;

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
  esc.writeMicroseconds(DRIBBLER_MIN_US);
  delay(3000);
  dribbler_iniciar_ligado();

  Serial.println("Robo iniciado");
}

void loop() {
  atualizar_gyro();
  ir_reader();
  atualizar_linha();
  monitor_serial();

  if (!manter_robo_sobre_linha()) {
    return;
  }

  if (ignorarFimCursoAteSoltar && !fim_curso_acionado()) {
    ignorarFimCursoAteSoltar = false;
  }

  if (fim_curso_acionado() && !bolaCapturada && !ignorarFimCursoAteSoltar) {
    bolaCapturada = true;

    stop_robot();
    dribbler_com_bola();
    delay(TEMPO_ENCAIXAR_BOLA_MS);

    if (!fim_curso_acionado()) {
      bolaCapturada = false;
      dribbler_procura();
      Serial.println("Bola escapou antes de encaixar. Voltando para procurar.");
      return;
    }

    Serial.println("Bola capturada. Virando para o sul...");
    if (!olhar_para_sul_com_bola()) {
      bolaCapturada = false;
      dribbler_procura();
      Serial.println("Bola escapou ao virar para o sul. Voltando para procurar.");
      return;
    }

    stop_robot();
    delay(150);

    Serial.println("Chutando pelo menor ultrassonico lateral...");
    if (!chutar_pelo_menor_ultrassonico()) {
      bolaCapturada = false;
      dribbler_procura();
      Serial.println("Bola escapou no chute. Voltando para procurar.");
      return;
    }

    stop_robot();
    dribbler_procura();
    bolaCapturada = false;
    ignorarFimCursoAteSoltar = true;

    Serial.println("Finalizado. Voltando para procurar a bolinha.");
    return;
  }

  if (bolaCapturada) {
    dribbler_com_bola();
    stop_robot();
    return;
  }

  dribbler_procura();
  seguir_bola_1_eixo();

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
  Serial.print(sensorLinhaPreta);
  Serial.print(" / Posicao: ");
  Serial.println(nome_posicao_linha(posicaoLinha));

  Serial.println("-------------------------------");
}

void ir_reader() {
  InfraredResult bola = InfraredSeeker::ReadAC();

  ballDirecao = bola.Direction;
}

void seguir_bola_1_eixo() {
  if (posicaoLinha != LINHA_CENTRO) {
    corrigir_para_centro_da_linha();
    return;
  }

  if (ballDirecao == 0) {
    stop_robot();
    return;
  }

  if (ballDirecao == 5) {
    stop_robot();
    return;
  }

  if (ballDirecao == 1 || ballDirecao == 2 || ballDirecao == 3 || ballDirecao == 4) {
    move_left_eixo(velocidadeEixo);
    return;
  }

  if (ballDirecao == 6 || ballDirecao == 7 || ballDirecao == 8 || ballDirecao == 9) {
    move_right_eixo(velocidadeEixo);
    return;
  }

  stop_robot();
}

bool manter_robo_sobre_linha() {
  if (posicaoLinha == LINHA_CENTRO) {
    return true;
  }

  corrigir_para_centro_da_linha();
  return false;
}

void corrigir_para_centro_da_linha() {
  if (posicaoLinha == LINHA_ESQUERDA) {
    move_right_eixo(velocidadeCorrecaoLinha);
    return;
  }

  if (posicaoLinha == LINHA_DIREITA) {
    move_left_eixo(velocidadeCorrecaoLinha);
    return;
  }

  stop_robot();

  if (millis() - ultimoTempoLinhaVista > TEMPO_BUSCA_LINHA_MS) {
    move_back(velocidadeBuscaLinha);
    delay(80);
    stop_robot();
    ultimoTempoLinhaVista = millis();
  }
}

bool fim_curso_acionado() {
  return digitalRead(fim_curso_pin) == LOW;
}

bool chutar_pelo_menor_ultrassonico() {
  if (!fim_curso_acionado()) {
    stop_robot();
    return false;
  }

  float distanciaEsquerda = ler_distancia_cm(echo_esquerda);
  delay(20);
  float distanciaDireita = ler_distancia_cm(echo_direita);

  Serial.print("Distancia esquerda: ");
  Serial.println(distanciaEsquerda);
  Serial.print("Distancia direita: ");
  Serial.println(distanciaDireita);

  bool girarParaEsquerda;

  if (distanciaEsquerda <= 0 && distanciaDireita <= 0) {
    girarParaEsquerda = true;
    Serial.println("Sem leitura lateral. Girando para esquerda por padrao.");
  } else if (distanciaDireita <= 0) {
    girarParaEsquerda = true;
    Serial.println("Direita sem leitura. Girando para esquerda.");
  } else if (distanciaEsquerda <= 0) {
    girarParaEsquerda = false;
    Serial.println("Esquerda sem leitura. Girando para direita.");
  } else if (distanciaEsquerda < distanciaDireita) {
    girarParaEsquerda = true;
    Serial.println("Esquerda menor. Girando para esquerda para chutar.");
  } else {
    girarParaEsquerda = false;
    Serial.println("Direita menor. Girando para direita para chutar.");
  }

  return girar_para_chutar(girarParaEsquerda);
}

bool girar_para_chutar(bool paraEsquerda) {
  unsigned long inicio = millis();

  dribbler_chute();

  while (millis() - inicio < TEMPO_GIRO_CHUTE_MS) {
    atualizar_gyro();

    if (verificar_linha_preta()) {
      inicio = millis();
      continue;
    }

    if (!fim_curso_acionado()) {
      stop_robot();
      return false;
    }

    if (paraEsquerda) {
      left_rotation(velocidadeGiroChute);
    } else {
      right_rotation(velocidadeGiroChute);
    }

    delay(10);
  }

  stop_robot();
  return true;
}

bool verificar_linha_preta() {
  atualizar_linha();

  if (!linhaPretaDetectada) {
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
    dribbler_com_bola();
  } else {
    dribbler_procura();
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

void atualizar_linha() {
  lerLDR();
  posicaoLinha = detectar_posicao_linha();
  linhaPretaDetectada = posicaoLinha != LINHA_PERDIDA;

  if (linhaPretaDetectada) {
    ultimoTempoLinhaVista = millis();
    digitalWrite(led_pin, HIGH);
  } else {
    digitalWrite(led_pin, LOW);
  }
}

PosicaoLinha detectar_posicao_linha() {
  int leiturasEsquerda = 0;
  int leiturasCentro = 0;
  int leiturasDireita = 0;
  sensorLinhaPreta = -1;

  for (int i = 0; i < 16; i++) {
    bool linhaDetectada;

    if (LINHA_VALOR_MENOR_QUE_QUADRA) {
      linhaDetectada = ldr[i] < LIMIAR_LINHA;
    } else {
      linhaDetectada = ldr[i] > LIMIAR_LINHA;
    }

    if (!linhaDetectada) {
      continue;
    }

    if (sensorLinhaPreta == -1) {
      sensorLinhaPreta = i;
    }

    if (i <= SENSOR_LINHA_ESQUERDA_FIM) {
      leiturasEsquerda++;
    } else if (i >= SENSOR_LINHA_CENTRO_INICIO && i <= SENSOR_LINHA_CENTRO_FIM) {
      leiturasCentro++;
    } else if (i >= SENSOR_LINHA_DIREITA_INICIO) {
      leiturasDireita++;
    }
  }

  if (leiturasCentro > 0) {
    return LINHA_CENTRO;
  }

  if (leiturasEsquerda > leiturasDireita) {
    return LINHA_ESQUERDA;
  }

  if (leiturasDireita > leiturasEsquerda) {
    return LINHA_DIREITA;
  }

  return LINHA_PERDIDA;
}

const char* nome_posicao_linha(PosicaoLinha posicao) {
  if (posicao == LINHA_ESQUERDA) {
    return "ESQUERDA";
  }

  if (posicao == LINHA_CENTRO) {
    return "CENTRO";
  }

  if (posicao == LINHA_DIREITA) {
    return "DIREITA";
  }

  return "PERDIDA";
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

bool olhar_para_sul_com_bola() {
  return girar_ate_angulo_com_bola(ANGULO_SUL, velocidadeGiro, 6000);
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

bool girar_ate_angulo_com_bola(float alvo, int velocidade, unsigned long tempoMaximo) {
  unsigned long inicio = millis();

  while (true) {
    atualizar_gyro();
    monitor_serial();

    if (!fim_curso_acionado()) {
      stop_robot();
      return false;
    }

    float erro = normalizar_angulo(alvo - anguloRobo);
    float erroAbs = abs(erro);

    if (erroAbs <= TOLERANCIA_NORTE) {
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

void dribbler_procura() {
  esc.writeMicroseconds(DRIBBLER_PROCURA_US);
  dribblerMicroAtual = DRIBBLER_PROCURA_US;
  dribblerLigado = true;
}

void dribbler_iniciar_ligado() {
  esc.writeMicroseconds(DRIBBLER_PROCURA_US);
  dribblerMicroAtual = DRIBBLER_PROCURA_US;
  dribblerLigado = true;
  delay(600);
  dribbler_procura();
}

void dribbler_com_bola() {
  esc.writeMicroseconds(DRIBBLER_COM_BOLA_US);
  dribblerMicroAtual = DRIBBLER_COM_BOLA_US;
  dribblerLigado = true;
}

void dribbler_chute() {
  esc.writeMicroseconds(DRIBBLER_CHUTE_US);
  dribblerMicroAtual = DRIBBLER_CHUTE_US;
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

void move_left_eixo(int v) {
  motor_1(HORARIO, v);
  motor_2(ANTI_HORARIO, v);
  motor_3(ANTI_HORARIO, v);
  motor_4(ANTI_HORARIO, v);
}

void move_right_eixo(int v) {
  motor_1(ANTI_HORARIO, v);
  motor_2(HORARIO, v);
  motor_3(HORARIO, v);
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
