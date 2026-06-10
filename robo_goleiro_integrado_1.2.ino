#include <Wire.h>
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

#define MOV_PARADO 0
#define MOV_FRENTE 1
#define MOV_TRAS 2
#define MOV_ESQUERDA 3
#define MOV_DIREITA 4
#define MOV_GIRO_ESQUERDA 5
#define MOV_GIRO_DIREITA 6

#define MPU_ADDR 0x68

int velocidadeFrente = 170;
int velocidadeGiro = 95;
int velocidadeBusca = 85;
int velocidadeDesvio = 120;
int velocidadeGoleiroLado = 140;
int velocidadeGoleiroAjuste = 130;
int velocidadeRecuperarLinha = 110;

int ballDirecao = 0;
int ballIntens = 0;

String estadoRobo = "Iniciando";

float gyroZOffset = 0;
float anguloRobo = 0;
unsigned long ultimoTempoGyro = 0;
unsigned long ultimoPrintSerial = 0;
const unsigned long INTERVALO_SERIAL = 1000;
int telaSerial = 0;

const int TOLERANCIA_NORTE = 5;
const int DISTANCIA_PARAR_CM = 25;
const int DISTANCIA_PAREDE_PERTO_CM = 12;
const unsigned long TEMPO_DESVIO_PAREDE_MS = 350;

const bool FIM_CURSO_ATIVO_LOW = true;
const unsigned long DEBOUNCE_FIM_CURSO_MS = 35;

bool GYRO_INVERTIDO = false;

int ldr[16];
bool linhaBrancaDetectada = false;
int sensorBranco = -1;
int lineAngle = 500;
int ultimoMovimentoRobo = MOV_PARADO;
int movimentoAntesDePerderLinha = MOV_PARADO;
unsigned long momentoLinhaPerdida = 0;

bool linha[16];

const int BALL_CENTER = 5;
const int MIN_SENSORES_LINHA_OK = 2;

float ultimaDistanciaFrenteCm = 0;
float ultimaDistanciaEsquerdaCm = 0;
float ultimaDistanciaDireitaCm = 0;

float vectorX[16] = {
   1.0000,  0.9239,  0.7071,  0.3827,
   0.0000, -0.3827, -0.7071, -0.9239,
  -1.0000, -0.9239, -0.7071, -0.3827,
   0.0000,  0.3827,  0.7071,  0.9239
};

float vectorY[16] = {
   0.0000,  0.3827,  0.7071,  0.9239,
   1.0000,  0.9239,  0.7071,  0.3827,
   0.0000, -0.3827, -0.7071, -0.9239,
  -1.0000, -0.9239, -0.7071, -0.3827
};

const int LIMIAR_BRANCO = 500;
const bool BRANCO_VALOR_MAIOR = false;

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

  estadoRobo = "Goleiro na linha";
  imprimir_menu_serial();
  Serial.println("Robo iniciado em modo goleiro com LDR vetorial");
}

void loop() {
  atualizar_gyro();
  ir_reader();
  monitor_serial();

  controle_goleiro_com_ldr();

  delay(25);
}

void controle_goleiro_com_ldr() {
  atualizar_sensores_linha_goleiro();
  imprimir_sensores_linha_goleiro();

  if (!linhaBrancaDetectada) {
    estadoRobo = "Recuperando linha";
    recuperar_linha_perdida();
    return;
  }

  momentoLinhaPerdida = 0;

  if (esta_na_posicao_certa_da_linha() || sensores_linha_ativos() >= MIN_SENSORES_LINHA_OK) {
    estadoRobo = "Goleiro seguindo bola";
    seguir_bola_eixo_x();
    return;
  }

  estadoRobo = "Centralizando na linha";
  centralizar_na_linha_curva();
}

bool esta_na_posicao_certa_da_linha() {
  bool ladoDireito = linha[2] || linha[3];
  bool ladoEsquerdo = linha[13] || linha[14];

  return ladoDireito && ladoEsquerdo;
}

bool linha_na_frente() {
  return linha[0] || linha[1] || linha[15];
}

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

int sensores_linha_ativos() {
  int total = 0;

  for (int i = 0; i < 16; i++) {
    if (linha[i]) {
      total++;
    }
  }

  return total;
}

void centralizar_na_linha_curva() {
  if (lineAngle == 500) {
    stop_robot();
    return;
  }

  if (lineAngle >= 45 && lineAngle < 135) {
    move_foward(velocidadeGoleiroAjuste);
  } else if (lineAngle >= 225 && lineAngle < 315) {
    move_back(velocidadeGoleiroAjuste);
  } else if (lineAngle >= 135 && lineAngle < 225) {
    move_left(velocidadeGoleiroAjuste);
  } else {
    move_right(velocidadeGoleiroAjuste);
  }
}

void recuperar_linha_perdida() {
  if (momentoLinhaPerdida == 0) {
    momentoLinhaPerdida = millis();
    movimentoAntesDePerderLinha = ultimoMovimentoRobo;
  }

  if (millis() - momentoLinhaPerdida > 1500) {
    stop_robot();
    return;
  }

  if (movimentoAntesDePerderLinha == MOV_DIREITA) {
    move_left(velocidadeRecuperarLinha);
  } else if (movimentoAntesDePerderLinha == MOV_ESQUERDA) {
    move_right(velocidadeRecuperarLinha);
  } else if (movimentoAntesDePerderLinha == MOV_FRENTE) {
    move_back(velocidadeRecuperarLinha);
  } else if (movimentoAntesDePerderLinha == MOV_TRAS) {
    move_foward(velocidadeRecuperarLinha);
  } else if (movimentoAntesDePerderLinha == MOV_GIRO_DIREITA) {
    left_rotation(velocidadeRecuperarLinha);
  } else if (movimentoAntesDePerderLinha == MOV_GIRO_ESQUERDA) {
    right_rotation(velocidadeRecuperarLinha);
  } else {
    move_back(velocidadeRecuperarLinha);
  }
}

void seguir_bola_eixo_x() {
  if (ballDirecao == 0) {
    stop_robot();
    return;
  }

  if (ballDirecao >= 4 && ballDirecao <= 6) {
    stop_robot();
    return;
  }

  if (ballDirecao < BALL_CENTER) {
    move_left(velocidadeGoleiroLado);
    return;
  }

  if (ballDirecao > BALL_CENTER) {
    move_right(velocidadeGoleiroLado);
    return;
  }
}

bool leitura_eh_linha(int leitura) {
  if (BRANCO_VALOR_MAIOR) {
    return leitura > LIMIAR_BRANCO;
  }

  return leitura < LIMIAR_BRANCO;
}

void atualizar_sensores_linha_goleiro() {
  lerLDR();
  linhaBrancaDetectada = false;
  sensorBranco = -1;

  for (int i = 0; i < 16; i++) {
    linha[i] = leitura_eh_linha(ldr[i]);

    if (linha[i]) {
      linhaBrancaDetectada = true;
      if (sensorBranco == -1) {
        sensorBranco = i;
      }
    }
  }

  calcularDirecaoLinha();
}

void imprimir_sensores_linha_goleiro() {
  static unsigned long ultimoPrintLinha = 0;

  if (millis() - ultimoPrintLinha < 300) {
    return;
  }

  ultimoPrintLinha = millis();

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

void monitor_serial() {
  atualizar_serial();
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
  Serial.print("| Linha detectada: ");
  Serial.println(linhaBrancaDetectada ? "SIM" : "NAO");
  Serial.print("| Sensor na linha: ");
  Serial.println(sensorBranco);
  Serial.print("| Angulo linha: ");
  Serial.println(lineAngle);
  Serial.print("| Frente: ");
  Serial.print(ultimaDistanciaFrenteCm);
  Serial.println(" cm");
  Serial.print("| Esquerda: ");
  Serial.print(ultimaDistanciaEsquerdaCm);
  Serial.println(" cm");
  Serial.print("| Direita: ");
  Serial.print(ultimaDistanciaDireitaCm);
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
  atualizar_ultrassonicos();

  Serial.println("+-------------------------------+");
  Serial.println("|       SENSOR ULTRASSONICO     |");
  Serial.println("+-------------------------------+");

  Serial.print("| Frente: ");
  Serial.print(ultimaDistanciaFrenteCm);
  Serial.println(" cm");

  Serial.print("| Esquerda: ");
  Serial.print(ultimaDistanciaEsquerdaCm);
  Serial.println(" cm");

  Serial.print("| Direita: ");
  Serial.print(ultimaDistanciaDireitaCm);
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
  atualizar_sensores_linha_goleiro();

  Serial.println("+-------------------------------+");
  Serial.println("|             LDR               |");
  Serial.println("+-------------------------------+");

  for (int i = 0; i < 16; i++) {
    Serial.print("| LDR ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(ldr[i]);
    Serial.print(" / Limiar: ");
    Serial.print(LIMIAR_BRANCO);
    Serial.print(" / Linha: ");
    Serial.println(linha[i] ? "SIM" : "NAO");
  }

  Serial.print("| Linha detectada: ");
  Serial.println(linhaBrancaDetectada ? "SIM" : "NAO");

  Serial.print("| Sensor na linha: ");
  Serial.println(sensorBranco);

  Serial.print("| Angulo linha: ");
  Serial.println(lineAngle);

  Serial.println("+-------------------------------+");
}

void ir_reader() {
  InfraredResult bola = InfraredSeeker::ReadAC();

  ballDirecao = bola.Direction;
  ballIntens = bola.Strength;
}

bool leitura_fim_curso_bruta_acionada() {
  int leitura = digitalRead(fim_curso_pin);

  if (FIM_CURSO_ATIVO_LOW) {
    return leitura == LOW;
  }

  return leitura == HIGH;
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

bool verificar_linha() {
  lerLDR();
  linhaBrancaDetectada = detectarBranco();
  calcularDirecaoLinha();

  if (!linhaBrancaDetectada) {
    digitalWrite(led_pin, LOW);
    return false;
  }

  digitalWrite(led_pin, HIGH);
  estadoRobo = "Linha branca detectada";
  Serial.println("Linha branca detectada. Indo para o sentido oposto.");
  afastar_da_linha();
  return true;
}

void calcularDirecaoLinha() {
  float somaX = 0;
  float somaY = 0;
  int sensoresAtivos = 0;

  for (int i = 0; i < 16; i++) {
    bool sensorNaLinha = leitura_eh_linha(ldr[i]);

    if (sensorNaLinha) {
      somaX += vectorX[i];
      somaY += vectorY[i];
      sensoresAtivos++;
    }
  }

  if (sensoresAtivos == 0) {
    lineAngle = 500;
    return;
  }

  lineAngle = atan2(somaY, somaX) * 180.0 / PI;

  if (lineAngle < 0) {
    lineAngle += 360;
  }
}

void afastar_da_linha() {
  if (lineAngle == 500) {
    stop_robot();
    return;
  }

  if (lineAngle >= 45 && lineAngle < 135) {
    move_back(velocidadeDesvio);
  } else if (lineAngle >= 225 && lineAngle < 315) {
    move_foward(velocidadeDesvio);
  } else if (lineAngle >= 135 && lineAngle < 225) {
    move_right(velocidadeDesvio);
  } else {
    move_left(velocidadeDesvio);
  }

  delay(350);
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

bool detectarBranco() {
  sensorBranco = -1;

  for (int i = 0; i < 16; i++) {
    bool linhaDetectada = leitura_eh_linha(ldr[i]);

    if (linhaDetectada) {
      sensorBranco = i;
      return true;
    }
  }

  return false;
}

void atualizar_ultrassonicos() {
  ultimaDistanciaFrenteCm = ler_distancia_cm(echo_frente);
  delay(25);

  ultimaDistanciaEsquerdaCm = ler_distancia_cm(echo_esquerda);
  delay(25);

  ultimaDistanciaDireitaCm = ler_distancia_cm(echo_direita);
  delay(25);
}

bool distancia_perto(float distancia, int limiteCm) {
  return distancia > 0 && distancia <= limiteCm;
}

bool parede_muito_perto() {
  atualizar_ultrassonicos();

  return distancia_perto(ultimaDistanciaFrenteCm, DISTANCIA_PAREDE_PERTO_CM) ||
         distancia_perto(ultimaDistanciaEsquerdaCm, DISTANCIA_PAREDE_PERTO_CM) ||
         distancia_perto(ultimaDistanciaDireitaCm, DISTANCIA_PAREDE_PERTO_CM);
}

void desviar_da_parede() {
  digitalWrite(led_pin, HIGH);
  estadoRobo = "Desviando da parede";

  if (distancia_perto(ultimaDistanciaFrenteCm, DISTANCIA_PAREDE_PERTO_CM)) {
    Serial.println("Parede na frente. Dando re.");
    move_back(velocidadeDesvio);
  } else if (distancia_perto(ultimaDistanciaEsquerdaCm, DISTANCIA_PAREDE_PERTO_CM)) {
    Serial.println("Parede na esquerda. Indo para a direita.");
    move_right(velocidadeDesvio);
  } else if (distancia_perto(ultimaDistanciaDireitaCm, DISTANCIA_PAREDE_PERTO_CM)) {
    Serial.println("Parede na direita. Indo para a esquerda.");
    move_left(velocidadeDesvio);
  }

  delay(TEMPO_DESVIO_PAREDE_MS);
  stop_robot();
  digitalWrite(led_pin, LOW);
  delay(100);
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

float erro_para_norte() {
  return normalizar_angulo(0 - anguloRobo);
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
  ultimoMovimentoRobo = MOV_FRENTE;
  motor_1(ANTI_HORARIO, v);
  motor_2(ANTI_HORARIO, v);
  motor_3(HORARIO, v);
  motor_4(ANTI_HORARIO, v);
}

void move_back(int v) {
  ultimoMovimentoRobo = MOV_TRAS;
  motor_1(HORARIO, v);
  motor_2(HORARIO, v);
  motor_3(ANTI_HORARIO, v);
  motor_4(HORARIO, v);
}

void move_left(int v) {
  ultimoMovimentoRobo = MOV_ESQUERDA;
  motor_1(HORARIO, v);
  motor_2(ANTI_HORARIO, v);
  motor_3(ANTI_HORARIO, v);
  motor_4(ANTI_HORARIO, v);
}

void move_right(int v) {
  ultimoMovimentoRobo = MOV_DIREITA;
  motor_1(ANTI_HORARIO, v);
  motor_2(HORARIO, v);
  motor_3(HORARIO, v);
  motor_4(HORARIO, v);
}

void left_rotation(int v) {
  ultimoMovimentoRobo = MOV_GIRO_ESQUERDA;
  motor_1(ANTI_HORARIO, v);
  motor_2(HORARIO, v);
  motor_3(ANTI_HORARIO, v);
  motor_4(ANTI_HORARIO, v);
}

void right_rotation(int v) {
  ultimoMovimentoRobo = MOV_GIRO_DIREITA;
  motor_1(HORARIO, v);
  motor_2(ANTI_HORARIO, v);
  motor_3(HORARIO, v);
  motor_4(HORARIO, v);
}

void stop_robot() {
  ultimoMovimentoRobo = MOV_PARADO;
  motor_1(PARADO, 0);
  motor_2(PARADO, 0);
  motor_3(PARADO, 0);
  motor_4(PARADO, 0);
}
