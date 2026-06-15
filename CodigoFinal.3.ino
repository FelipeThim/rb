#include <Wire.h>
#include <Servo.h>
#include <HTInfraredSeeker.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

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

Servo esc;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

int velocidadeFrente = 210;
int velocidadeGiro = 110;
int velocidadeBusca = 150;
int velocidadeReLinha = 210;
int velocidadeGiroComBola = 200;

int ballDirecao = 0;

bool bolaCapturada = false;
bool ignorarFimCursoAteSoltar = false;
bool dribblerLigado = false;
int dribblerMicroAtual = -1;

float anguloRobo = 0;
unsigned long ultimoPrintSerial = 0;
float bnoHeadingInicial = 0;
float anguloBnoFiltrado = 0;
bool bnoPrimeiraLeitura = true;
float bnoHeadingBruto = 0;
float bnoAnguloRelativo = 0;

// ===== FLAGS / CONFIGURACOES =====
const bool LINHA_VALOR_MENOR_QUE_QUADRA = false;
const bool TESTE_MOTORES_ATIVO = false;
const bool VERIFICAR_LINHA_DURANTE_GIRO_COM_BOLA = false;
const bool GIRAR_COM_BOLA_PARA_LADO_MAIOR = false;
const bool INVERTER_SENTIDO_GIRO_COM_BOLA = false;

// ===== TEMPOS =====
const unsigned long TEMPO_ENCAIXAR_BOLA_MS = 750;
const unsigned long TEMPO_GIRO_COM_BOLA_MS = 800;
const unsigned long TEMPO_RE_LINHA_MS = 450;
const unsigned long TEMPO_PARADO_NO_SUL_MS = 1000;
const unsigned long INTERVALO_LOOP_MS = 10;
const unsigned long TIMEOUT_ULTRASSONICO_US = 12000;
const int AMOSTRAS_ULTRASSONICO_LATERAL = 5;

// ===== ANGULOS =====
const int TOLERANCIA_ANGULO = 10;
const int ANGULO_NORTE = 0;
const int ANGULO_SUL = 180;

// ===== VELOCIDADES GERAIS =====
const int VELOCIDADE_GIRO_MINIMA = 180;
const int VELOCIDADE_ALINHAMENTO_COM_BOLA = 150; // anteriormente 170
const int VELOCIDADE_CHUTE_GIRO = 220;

// ===== DRIBBLER =====
#define DRIBBLER_ESC_PIN 9
#define DRIBBLER_PARADO_US 1000
#define DRIBBLER_VELOCIDADE_PROCURA_US 1140
#define DRIBBLER_VELOCIDADE_COM_BOLA_US 1200
#define DRIBBLER_VELOCIDADE_PARADO_NO_SUL_US 1100

// ===== VELOCIDADES DA LOGICA COM BOLA =====
const int VELOCIDADE_GIRO_SUL_COM_BOLA = 130;
const int VELOCIDADE_GIRO_NORTE_LENTO = 170;
const int VELOCIDADE_GIRO_NORTE_RAPIDO = 190;
const int ANGULO_RESTANTE_PARA_GIRO_NORTE_RAPIDO = 135;
const int VELOCIDADE_CHICOTE_COM_BOLA = 220;
const int ANGULO_CHICOTE_GOL = 40;
const unsigned long TEMPO_MAXIMO_CHICOTE_MS = 1500;

// ===== CONTROLE DO GIRO =====
const float KP_GIRO_ANGULO = 3.0;

// ================================== //

const bool DEBUG_SERIAL_ATIVO = true;
const bool BNO_INVERTER_ANGULO = false;
const bool BNO_USAR_MODO_IMUPLUS = true;
const float BNO_FILTRO_ALPHA = 0.35;

// ================================== //

const int LIMIAR_LINHA = 150; // alterar para sensor de linha na quadra
const float BNO_ZONA_MORTA_GRAUS = 0.5;
const bool GIRO_SUL_PARA_ESQUERDA = true;
const bool GIRO_NORTE_PARA_ESQUERDA = false;

// ================================== //


bool GYRO_INVERTIDO = false;

int ldr[16];
bool linhaPretaDetectada = false;
int sensorLinhaPreta = -1;

float ultimaDistanciaFrenteCm = 0;
float ultimaDistanciaEsquerdaCm = 0;
float ultimaDistanciaDireitaCm = 0;
bool leituraSensorBola = false;

template <typename T>
void debug_print(T valor) {
  if (DEBUG_SERIAL_ATIVO) {
    Serial.print(valor);
  }
}

template <typename T>
void debug_println(T valor) {
  if (DEBUG_SERIAL_ATIVO) {
    Serial.println(valor);
  }
}

void setup() {
  if (DEBUG_SERIAL_ATIVO) {
    Serial.begin(115200);
  }

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

  bno055_iniciar();
  zerar_angulo_robo();

  stop_robot();

  esc.attach(DRIBBLER_ESC_PIN);
  dribbler_parado();
  delay(3000);

  if (TESTE_MOTORES_ATIVO) {
    testar_motores_e_giro();
  }

  dribbler_velocidade_procura();

  debug_println("Robo iniciado");
}

void loop() {
  atualizar_gyro();

  ir_reader();
  monitor_serial();

  bool fimCursoAgora = fim_curso_acionado();

  if (ignorarFimCursoAteSoltar && !fimCursoAgora) {
    ignorarFimCursoAteSoltar = false;
  }

  if (fimCursoAgora && !bolaCapturada && !ignorarFimCursoAteSoltar) {
    bolaCapturada = true;

    debug_println("Fim de curso acionado uma vez. Iniciando sequencia com bola...");
    debug_println("Esperando a bola encaixar antes de acelerar o dribbler...");
    esperar_bola_encaixar();

    dribbler_velocidade_com_bola();
    debug_println("Dribbler acelerado. Esperando a bola encaixar melhor...");
    esperar_bola_encaixar();

    debug_println("Bola capturada. Virando devagar para o sul / 180 graus...");
    debug_print("Angulo antes do sul: ");
    debug_println(anguloRobo);
    if (!olhar_para_sul_com_bola_lento()) {
      debug_println("Bola escapou ao alinhar no sul. Voltando para procurar.");
      dribbler_velocidade_procura();
      bolaCapturada = false;
      ignorarFimCursoAteSoltar = true;
      return;
    }

    stop_robot();
    debug_print("Parado no sul. Angulo atual: ");
    debug_println(anguloRobo);
    dribbler_velocidade_parado_no_sul();
    delay(TEMPO_PARADO_NO_SUL_MS);

    debug_println("Lendo ultrassonicos parado no sul para escolher o lado do chicote...");
    if (!chicotear_pelo_ultrassonico_lateral()) {
      debug_println("Falha ao fazer chicote pelo ultrassonico. Voltando para procurar.");
      dribbler_velocidade_procura();
      bolaCapturada = false;
      ignorarFimCursoAteSoltar = true;
      return;
    }

    dribbler_velocidade_procura();
    bolaCapturada = false;
    ignorarFimCursoAteSoltar = true;

    debug_println("Finalizado. Voltando para procurar a bolinha.");
    return;
  }

  if (verificar_linha_preta()) {
    return;
  }

  if (bolaCapturada) {
    dribbler_velocidade_com_bola();
    stop_robot();
    return;
  }

  if (!fimCursoAgora) {
    dribbler_velocidade_procura();
  }

  seguir_bola_simples();

  delay(INTERVALO_LOOP_MS);
}

void monitor_serial() {
  if (!DEBUG_SERIAL_ATIVO) {
    return;
  }

  if (millis() - ultimoPrintSerial < 500) {
    return;
  }

  ultimoPrintSerial = millis();
  atualizar_ultrassonicos();

  debug_println("-------------------------------");

  debug_print("Direcao IR: ");
  debug_println(ballDirecao);

  debug_print("Fim de curso: ");
  debug_print(fim_curso_acionado() ? "ACIONADO" : "SOLTO");
  debug_print(" / Pino: ");
  debug_println(digitalRead(fim_curso_pin) == LOW ? "LOW" : "HIGH");

  debug_print("BNO heading bruto: ");
  debug_println(bnoHeadingBruto);

  debug_print("BNO heading inicial: ");
  debug_println(bnoHeadingInicial);

  debug_print("BNO angulo relativo: ");
  debug_println(bnoAnguloRelativo);

  debug_print("BNO angulo filtrado/usado: ");
  debug_println(anguloRobo);

  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  debug_print("Calibracao BNO SYS/GYR/ACC/MAG: ");
  debug_print(sys);
  debug_print("/");
  debug_print(gyro);
  debug_print("/");
  debug_print(accel);
  debug_print("/");
  debug_println(mag);

  debug_print("Erro para Norte / 0 graus: ");
  debug_println(erro_para_norte());

  debug_print("Erro para Sul / 180 graus: ");
  debug_println(erro_para_sul());

  debug_print("Dribbler: ");
  debug_print(dribblerLigado ? "LIGADO" : "PARADO");
  debug_print(" / ");
  debug_print(dribblerMicroAtual);
  debug_println(" us");

  debug_print("Linha preta: ");
  debug_print(linhaPretaDetectada ? "SIM" : "NAO");
  debug_print(" / Sensor: ");
  debug_println(sensorLinhaPreta);

  debug_print("Ultrassonico frente: ");
  debug_print(ultimaDistanciaFrenteCm);
  debug_println(" cm");

  debug_print("Ultrassonico esquerda: ");
  debug_print(ultimaDistanciaEsquerdaCm);
  debug_println(" cm");

  debug_print("Ultrassonico direita: ");
  debug_print(ultimaDistanciaDireitaCm);
  debug_println(" cm");

  debug_println("-------------------------------");
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

  if (ballDirecao == 5) {
    move_foward(velocidadeFrente);
    return;
  }

  if (ballDirecao == 1 || ballDirecao == 2 || ballDirecao == 3 || ballDirecao == 4 ) {
    left_rotation(velocidadeGiro);
    return;
  }

  if ( ballDirecao == 6 || ballDirecao == 7 || ballDirecao == 8 || ballDirecao == 9) {
    right_rotation(velocidadeGiro);
    return;
  }

  stop_robot();
}

bool fim_curso_acionado() {
  leituraSensorBola = digitalRead(fim_curso_pin) == LOW;
  return leituraSensorBola;
}

void esperar_bola_encaixar() {
  unsigned long inicio = millis();

  while (millis() - inicio < TEMPO_ENCAIXAR_BOLA_MS) {
    atualizar_gyro();

    if (verificar_linha_preta()) {
      inicio = millis();
      continue;
    }

    stop_robot();
    delay(20);
  }
}

bool girar_pelo_ultrassonico_lateral() {
  if (!fim_curso_acionado()) {
    stop_robot();
    dribbler_velocidade_procura();
    return false;
  }

  ultimaDistanciaEsquerdaCm = medir_media_ultrassonico_valida(echo_esquerda, AMOSTRAS_ULTRASSONICO_LATERAL);
  delay(5);
  ultimaDistanciaDireitaCm = medir_media_ultrassonico_valida(echo_direita, AMOSTRAS_ULTRASSONICO_LATERAL);
  stop_robot();

  debug_print("Media esquerda valida: ");
  debug_println(ultimaDistanciaEsquerdaCm);
  debug_print("Media direita valida: ");
  debug_println(ultimaDistanciaDireitaCm);

  if (ultimaDistanciaDireitaCm <= 0 && ultimaDistanciaEsquerdaCm <= 0) {
    debug_println("Laterais sem leitura. Girando para a esquerda por padrao.");
    return girar_esquerda_com_bola_por_tempo(TEMPO_GIRO_COM_BOLA_MS);
  }

  bool escolherLadoDireito;

  if (ultimaDistanciaDireitaCm <= 0) {
    escolherLadoDireito = false;
  } else if (ultimaDistanciaEsquerdaCm <= 0) {
    escolherLadoDireito = true;
  } else if (GIRAR_COM_BOLA_PARA_LADO_MAIOR) {
    escolherLadoDireito = ultimaDistanciaDireitaCm > ultimaDistanciaEsquerdaCm;
  } else {
    escolherLadoDireito = ultimaDistanciaDireitaCm < ultimaDistanciaEsquerdaCm;
  }

  bool girarParaEsquerda = escolherLadoDireito;

  if (INVERTER_SENTIDO_GIRO_COM_BOLA) {
    girarParaEsquerda = !girarParaEsquerda;
  }

  debug_print(GIRAR_COM_BOLA_PARA_LADO_MAIOR ? "Escolhendo lado maior: " : "Escolhendo lado menor: ");
  debug_println(escolherLadoDireito ? "direita" : "esquerda");
  debug_print("Sentido final do giro: ");
  debug_println(girarParaEsquerda ? "esquerda" : "direita");

  if (girarParaEsquerda) {
    return girar_esquerda_com_bola_por_tempo(TEMPO_GIRO_COM_BOLA_MS);
  }

  return girar_direita_com_bola_por_tempo(TEMPO_GIRO_COM_BOLA_MS);
}

bool chicotear_pelo_ultrassonico_lateral() {
  if (!fim_curso_acionado()) {
    stop_robot();
    dribbler_velocidade_procura();
    return false;
  }

  stop_robot();
  delay(200);
  atualizar_gyro();
  debug_print("Angulo no momento da leitura ultrassonica: ");
  debug_println(anguloRobo);

  ultimaDistanciaEsquerdaCm = medir_media_ultrassonico_valida(echo_esquerda, AMOSTRAS_ULTRASSONICO_LATERAL);
  delay(5);
  ultimaDistanciaDireitaCm = medir_media_ultrassonico_valida(echo_direita, AMOSTRAS_ULTRASSONICO_LATERAL);
  stop_robot();

  debug_print("Media esquerda valida: ");
  debug_println(ultimaDistanciaEsquerdaCm);
  debug_print("Media direita valida: ");
  debug_println(ultimaDistanciaDireitaCm);

  bool ladoMaiorDireito;

  if (ultimaDistanciaDireitaCm <= 0 && ultimaDistanciaEsquerdaCm <= 0) {
    debug_println("Laterais sem leitura. Chicote para a esquerda por padrao.");
    ladoMaiorDireito = false;
  } else if (ultimaDistanciaDireitaCm <= 0) {
    ladoMaiorDireito = false;
  } else if (ultimaDistanciaEsquerdaCm <= 0) {
    ladoMaiorDireito = true;
  } else {
    ladoMaiorDireito = ultimaDistanciaDireitaCm > ultimaDistanciaEsquerdaCm;
  }

  bool chicoteParaEsquerda = !ladoMaiorDireito;
  float alvoChicote;

  if (chicoteParaEsquerda) {
    alvoChicote = normalizar_angulo(ANGULO_SUL + ANGULO_CHICOTE_GOL);
  } else {
    alvoChicote = normalizar_angulo(ANGULO_SUL - ANGULO_CHICOTE_GOL);
  }

  debug_print("Lado maior: ");
  debug_println(ladoMaiorDireito ? "direita" : "esquerda");
  debug_print("Chicote para: ");
  debug_println(chicoteParaEsquerda ? "esquerda" : "direita");
  debug_print("Alvo do chicote: ");
  debug_println(alvoChicote);

  if (!girar_chicote_com_dribbler_ate_metade(alvoChicote, VELOCIDADE_CHICOTE_COM_BOLA, TEMPO_MAXIMO_CHICOTE_MS, chicoteParaEsquerda)) {
    return false;
  }

  debug_println("Voltando 40 graus para o sul para finalizar o chicote.");
  return girar_chicote_com_dribbler_ate_metade(ANGULO_SUL, VELOCIDADE_CHICOTE_COM_BOLA, TEMPO_MAXIMO_CHICOTE_MS, !chicoteParaEsquerda);
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
  debug_print("Linha preta detectada no sensor ");
  debug_println(sensorLinhaPreta);

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
  delay(5);

  ultimaDistanciaEsquerdaCm = ler_distancia_cm(echo_esquerda);
  delay(5);

  ultimaDistanciaDireitaCm = ler_distancia_cm(echo_direita);
  delay(5);
}

float medir_media_ultrassonico_valida(int echoPin, int quantidadeAmostras) {
  float soma = 0;
  int leiturasValidas = 0;

  for (int i = 0; i < quantidadeAmostras; i++) {
    float distancia = ler_distancia_cm(echoPin);

    if (distancia > 0) {
      soma += distancia;
      leiturasValidas++;
    }

    delay(10);
  }

  if (leiturasValidas == 0) {
    return 0;
  }

  return soma / leiturasValidas;
}

float ler_distancia_cm(int echoPin) {
  digitalWrite(trig_ultrassonico, LOW);
  delayMicroseconds(2);

  digitalWrite(trig_ultrassonico, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_ultrassonico, LOW);

  long duracao = pulseIn(echoPin, HIGH, TIMEOUT_ULTRASSONICO_US);

  if (duracao == 0) {
    return 0;
  }

  return duracao * 0.034 / 2;
}

void bno055_iniciar() {
  debug_println("Iniciando BNO055 9DOF...");

  if (!bno.begin()) {
    debug_println("ERRO: BNO055 nao encontrado. Teste endereco 0x29 ou confira SDA/SCL.");
    while (true) {
      stop_robot();
      delay(100);
    }
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  if (BNO_USAR_MODO_IMUPLUS) {
    bno.setMode(OPERATION_MODE_IMUPLUS);
    delay(50);
    debug_println("BNO055 em modo IMUPLUS");
  }

  debug_println("BNO055 iniciado");
}

float ler_heading_bno() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  return euler.x();
}

void zerar_angulo_robo() {
  bnoHeadingInicial = ler_heading_bno();
  anguloRobo = 0;
  anguloBnoFiltrado = 0;
  bnoPrimeiraLeitura = true;
  debug_println("Angulo do robo zerado em 0 graus pelo BNO055");
}

void atualizar_gyro() {
  float headingAtual = ler_heading_bno();
  float anguloRelativo = normalizar_angulo(headingAtual - bnoHeadingInicial);

  bnoHeadingBruto = headingAtual;

  if (BNO_INVERTER_ANGULO || GYRO_INVERTIDO) {
    anguloRelativo = -anguloRelativo;
  }

  bnoAnguloRelativo = normalizar_angulo(anguloRelativo);

  if (bnoPrimeiraLeitura) {
    anguloBnoFiltrado = anguloRelativo;
    bnoPrimeiraLeitura = false;
  }

  float diferenca = normalizar_angulo(anguloRelativo - anguloBnoFiltrado);

  if (abs(diferenca) > BNO_ZONA_MORTA_GRAUS) {
    anguloBnoFiltrado = normalizar_angulo(anguloBnoFiltrado + diferenca * BNO_FILTRO_ALPHA);
  }

  anguloRobo = normalizar_angulo(anguloBnoFiltrado);
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
  return normalizar_angulo(ANGULO_NORTE - anguloRobo);
}

float erro_para_sul() {
  return normalizar_angulo(ANGULO_SUL - anguloRobo);
}

int velocidade_giro_proporcional(float erroAbs, int velocidadeMaxima) {
  int velocidadeCalculada = (int)(erroAbs * KP_GIRO_ANGULO);
  return constrain(velocidadeCalculada, VELOCIDADE_GIRO_MINIMA, velocidadeMaxima);
}

void olhar_para_sul() {
  girar_ate_angulo(ANGULO_SUL, velocidadeGiro, 6000);
}

void olhar_para_norte() {
  girar_ate_angulo(ANGULO_NORTE, velocidadeGiro, 6000);
}

bool olhar_para_sul_com_bola() {
  return girar_ate_angulo_com_bola(ANGULO_SUL, velocidadeGiroComBola, 6000);
}

bool olhar_para_norte_com_bola() {
  return girar_ate_angulo_com_bola(ANGULO_NORTE, velocidadeGiroComBola, 6000);
}

bool olhar_para_sul_com_bola_lento() {
  return girar_ate_angulo_com_bola_fixo(ANGULO_SUL, VELOCIDADE_GIRO_SUL_COM_BOLA, 7000);
}

bool olhar_para_norte_duas_etapas_com_bola() {
  unsigned long inicio = millis();

  dribbler_velocidade_com_bola();

  while (true) {
    atualizar_gyro();

    if (VERIFICAR_LINHA_DURANTE_GIRO_COM_BOLA && verificar_linha_preta()) {
      continue;
    }

    monitor_serial();

    float erro = normalizar_angulo(ANGULO_NORTE - anguloRobo);
    float erroAbs = abs(erro);

    if (erroAbs <= TOLERANCIA_ANGULO) {
      debug_println("Norte alcancado em duas etapas.");
      stop_robot();
      return true;
    }

    bool giroNorteRapido = erroAbs <= ANGULO_RESTANTE_PARA_GIRO_NORTE_RAPIDO;
    int velocidadeAtual = giroNorteRapido ? VELOCIDADE_GIRO_NORTE_RAPIDO : VELOCIDADE_GIRO_NORTE_LENTO;

    if (giroNorteRapido) {
      dribbler_parado();
    } else {
      dribbler_velocidade_com_bola();
    }

    if (GIRO_NORTE_PARA_ESQUERDA) {
      left_rotation(velocidadeAtual);
    } else {
      right_rotation(velocidadeAtual);
    }

    if (millis() - inicio > 7000) {
      stop_robot();
      return false;
    }

    delay(10);
  }
}

bool girar_ate_angulo_com_bola_fixo_sentido(float alvo, int velocidade, unsigned long tempoMaximo, bool paraEsquerda) {
  unsigned long inicio = millis();

  dribbler_velocidade_com_bola();

  while (true) {
    atualizar_gyro();

    if (VERIFICAR_LINHA_DURANTE_GIRO_COM_BOLA && verificar_linha_preta()) {
      continue;
    }

    monitor_serial();

    float erro = normalizar_angulo(alvo - anguloRobo);
    float erroAbs = abs(erro);

    if (erroAbs <= TOLERANCIA_ANGULO) {
      debug_print("BNO alvo ok: ");
      debug_println(alvo);
      stop_robot();
      return true;
    }

    if (paraEsquerda) {
      left_rotation(velocidade);
    } else {
      right_rotation(velocidade);
    }

    if (millis() - inicio > tempoMaximo) {
      stop_robot();
      return false;
    }

    delay(10);
  }
}

bool girar_chicote_com_dribbler_ate_metade(float alvo, int velocidade, unsigned long tempoMaximo, bool paraEsquerda) {
  unsigned long inicio = millis();

  atualizar_gyro();
  float erroInicialAbs = abs(normalizar_angulo(alvo - anguloRobo));
  float erroParaPararDribbler = erroInicialAbs / 2.0;

  dribbler_velocidade_com_bola();

  while (true) {
    atualizar_gyro();

    if (VERIFICAR_LINHA_DURANTE_GIRO_COM_BOLA && verificar_linha_preta()) {
      continue;
    }

    monitor_serial();

    float erro = normalizar_angulo(alvo - anguloRobo);
    float erroAbs = abs(erro);

    if (erroAbs <= TOLERANCIA_ANGULO) {
      debug_print("BNO alvo ok: ");
      debug_println(alvo);
      stop_robot();
      return true;
    }

    if (erroAbs <= erroParaPararDribbler) {
      dribbler_parado();
    } else {
      dribbler_velocidade_com_bola();
    }

    if (paraEsquerda) {
      left_rotation(velocidade);
    } else {
      right_rotation(velocidade);
    }

    if (millis() - inicio > tempoMaximo) {
      stop_robot();
      return false;
    }

    delay(10);
  }
}

bool girar_ate_angulo_com_bola_fixo(float alvo, int velocidade, unsigned long tempoMaximo) {
  unsigned long inicio = millis();

  dribbler_velocidade_com_bola();

  while (true) {
    atualizar_gyro();

    if (VERIFICAR_LINHA_DURANTE_GIRO_COM_BOLA && verificar_linha_preta()) {
      continue;
    }

    monitor_serial();

    float erro = normalizar_angulo(alvo - anguloRobo);
    float erroAbs = abs(erro);

    if (erroAbs <= TOLERANCIA_ANGULO) {
      debug_print("BNO alvo ok: ");
      debug_println(alvo);
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
      return false;
    }

    delay(10);
  }
}

void girar_ate_angulo(float alvo, int velocidade, unsigned long tempoMaximo) {
  unsigned long inicio = millis();

  while (true) {
    atualizar_gyro();

    if (VERIFICAR_LINHA_DURANTE_GIRO_COM_BOLA && verificar_linha_preta()) {
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

    if (VERIFICAR_LINHA_DURANTE_GIRO_COM_BOLA && verificar_linha_preta()) {
      continue;
    }

    monitor_serial();

    float erro = normalizar_angulo(alvo - anguloRobo);
    float erroAbs = abs(erro);

    if (erroAbs <= TOLERANCIA_ANGULO) {
      debug_print("BNO alvo ok: ");
      debug_println(alvo);
      stop_robot();
      return true;
    }

    int velocidadeAtual = velocidade_giro_proporcional(erroAbs, velocidade);

    if (erro > 0) {
      debug_print("BNO girando LEFT | alvo=");
      debug_print(alvo);
      debug_print(" erro=");
      debug_println(erro);
      left_rotation(velocidadeAtual);
    } else {
      debug_print("BNO girando RIGHT | alvo=");
      debug_print(alvo);
      debug_print(" erro=");
      debug_println(erro);
      right_rotation(velocidadeAtual);
    }

    if (millis() - inicio > tempoMaximo) {
      stop_robot();
      return false;
    }

    delay(10);
  }
}

void left_rotation_alinhamento_com_bola() {
  left_rotation(VELOCIDADE_ALINHAMENTO_COM_BOLA);
}

void right_rotation_alinhamento_com_bola() {
  right_rotation(VELOCIDADE_ALINHAMENTO_COM_BOLA);
}

void dribbler_velocidade_procura() {
  if (dribblerMicroAtual == DRIBBLER_VELOCIDADE_PROCURA_US) {
    return;
  }

  esc.writeMicroseconds(DRIBBLER_VELOCIDADE_PROCURA_US);
  dribblerMicroAtual = DRIBBLER_VELOCIDADE_PROCURA_US;
  dribblerLigado = true;
}

void dribbler_parado() {
  if (dribblerMicroAtual == DRIBBLER_PARADO_US) {
    return;
  }

  esc.writeMicroseconds(DRIBBLER_PARADO_US);
  dribblerMicroAtual = DRIBBLER_PARADO_US;
  dribblerLigado = false;
}

void dribbler_velocidade_com_bola() {
  if (dribblerMicroAtual == DRIBBLER_VELOCIDADE_COM_BOLA_US) {
    return;
  }

  esc.writeMicroseconds(DRIBBLER_VELOCIDADE_COM_BOLA_US);
  dribblerMicroAtual = DRIBBLER_VELOCIDADE_COM_BOLA_US;
  dribblerLigado = true;
}

void dribbler_velocidade_parado_no_sul() {
  if (dribblerMicroAtual == DRIBBLER_VELOCIDADE_PARADO_NO_SUL_US) {
    return;
  }

  esc.writeMicroseconds(DRIBBLER_VELOCIDADE_PARADO_NO_SUL_US);
  dribblerMicroAtual = DRIBBLER_VELOCIDADE_PARADO_NO_SUL_US;
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

void curva_frente_esquerda(int velocidadeBase, int correcao) {
  int velocidadeInterna = constrain(velocidadeBase - correcao, 0, 255);
  int velocidadeExterna = constrain(velocidadeBase, 0, 255);

  motor_1(ANTI_HORARIO, velocidadeExterna);
  motor_2(ANTI_HORARIO, velocidadeInterna);
  motor_3(HORARIO, velocidadeInterna);
  motor_4(ANTI_HORARIO, velocidadeExterna);
}

void curva_frente_direita(int velocidadeBase, int correcao) {
  int velocidadeInterna = constrain(velocidadeBase - correcao, 0, 255);
  int velocidadeExterna = constrain(velocidadeBase, 0, 255);

  motor_1(ANTI_HORARIO, velocidadeInterna);
  motor_2(ANTI_HORARIO, velocidadeExterna);
  motor_3(HORARIO, velocidadeExterna);
  motor_4(ANTI_HORARIO, velocidadeInterna);
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

void testar_motores_e_giro() {
  const int vTeste = 180;
  const unsigned long tempoMotorMs = 1200;
  const unsigned long pausaMs = 700;

  stop_robot();
  delay(1000);

  motor_1(HORARIO, vTeste);
  delay(tempoMotorMs);
  stop_robot();
  delay(pausaMs);

  motor_2(HORARIO, vTeste);
  delay(tempoMotorMs);
  stop_robot();
  delay(pausaMs);

  motor_3(HORARIO, vTeste);
  delay(tempoMotorMs);
  stop_robot();
  delay(pausaMs);

  motor_4(HORARIO, vTeste);
  delay(tempoMotorMs);
  stop_robot();
  delay(pausaMs);

  left_rotation(220);
  delay(1500);
  stop_robot();
  delay(pausaMs);

  right_rotation(220);
  delay(1500);
  stop_robot();
  delay(pausaMs);
}
