int lineAngle = 500;

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

void calcularDirecaoLinha()
{
  float somaX = 0;
  float somaY = 0;
  int sensoresAtivos = 0;

  for (int i = 0; i < 16; i++)
  {
    bool branco;

    if (BRANCO_VALOR_MAIOR)
    {
      branco = ldr[i] > LIMIAR_BRANCO;
    }
    else
    {
      branco = ldr[i] < LIMIAR_BRANCO;
    }

    if (branco)
    {
      somaX += vectorX[i];
      somaY += vectorY[i];
      sensoresAtivos++;
    }
  }

  if (sensoresAtivos == 0)
  {
    lineAngle = 500;
    return;
  }

  lineAngle = atan2(somaY, somaX) * 180.0 / PI;

  if (lineAngle < 0)
  {
    lineAngle += 360;
  }
}

void afastar_da_linha()
{
  if (lineAngle == 500)
  {
    stop_robot();
    return;
  }

  estadoRobo = "Fugindo da linha";

  if (lineAngle >= 45 && lineAngle < 135)
  {
    move_back();
  }
  else if (lineAngle >= 225 && lineAngle < 315)
  {
    move_foward();
  }
  else if (lineAngle >= 135 && lineAngle < 225)
  {
    move_right();
  }
  else
  {
    move_left();
  }

  delay(350);
  stop_robot();
  delay(100);
}
lerLDR();
linhaBrancaDetectada = detectarBranco();
calcularDirecaoLinha();

if (linhaBrancaDetectada)
{
  dribbler_parar();
  estadoRobo = "Linha branca detectada!";
  digitalWrite(led_pin, HIGH);
  afastar_da_linha();
  return;
}
else
{
  digitalWrite(led_pin, LOW);
}
Serial.print("| Angulo linha: ");
Serial.println(lineAngle);
