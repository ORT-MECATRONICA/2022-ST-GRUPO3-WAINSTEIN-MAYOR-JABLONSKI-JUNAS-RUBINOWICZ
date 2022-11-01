int getTemperaturaBmp() {
  return bmp.readTemperature();
}

int getPresionBmp() {
  int resultado = (bmp.readPressure() / 100);
  return resultado;
}


int getGasMq5() {
  int gasLP = analogRead(PIN_MQ5);
  int result = map(gasLP, 0, 4095, 0, 100);
  return result;
}

int getMonoxidoCarbonoMq7() {
  ////// Mq7 ////////
  int monoxidoDeCarbono = analogRead(PIN_MQ7);
  int result = map(monoxidoDeCarbono, 0, 4095, 0, 100);
  return result;
}

void alarma() {
  //int gas = 7;
  int gas = getGasMq5();
  switch (estadoBuzzer) {
    case BUZZER_OFF:
      if (gas > GAS_ALARMANTE) {
        Serial.println("encendido");
        ledcWrite(channel, SOUND_ON);
        estadoBuzzer = BUZZER_ON;
      }
      break;

    case BUZZER_ON:
      if (gas < GAS_ALARMANTE) {
        ledcWrite(channel, SOUND_OFF);
        estadoBuzzer = BUZZER_OFF;
      }
      break;
  }
}

int getLuzTemt() {
  int valorLuz = analogRead(PIN_TEMT6000);
  int porcentajeLuz = map(valorLuz, 0, 4095, 0, 100);
  return porcentajeLuz;
}

void displayText(String text) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(5, 28);
  display.println(text);
  display.display();
}

void displayoled() {

  switch (estadoDisplay) {
    case DISPLAY_TEMPERATURA:
      if (millis() < 5000) { //si recien se prende
        displayText("Temperatura: " + String(getTemperaturaBmp()));
        timerTerminado = false;
        //displayText("temperatura");
      }
      else if (timerTerminado) {
        estadoDisplay = DISPLAY_PRESION;
        timerTerminado = false;
        displayText("Presion: " + String(getPresionBmp()) + " hPa");
        //displayText("presion");
      }
      break;

    case DISPLAY_PRESION:
      if (timerTerminado) {
        estadoDisplay = DISPLAY_GAS;
        timerTerminado = false;
        displayText("Gas: " + String(getGasMq5()) + " %");
        //displayText("gas");
      }
      break;

    case DISPLAY_GAS:
      if (timerTerminado) {
        estadoDisplay = DISPLAY_LUZ;
        timerTerminado = false;
        displayText("Luz: " + String(getLuzTemt()) + " %");
        //displayText("luz");
      }
      break;

    case DISPLAY_LUZ:
      if (timerTerminado) {
        estadoDisplay = DISPLAY_CARBONO;
        timerTerminado = false;
        displayText("Carbono: " + String(getMonoxidoCarbonoMq7()) + " %");
        //displayText("carbono");
      }
      break;

    case DISPLAY_CARBONO:
      if (timerTerminado) {
        estadoDisplay = DISPLAY_TEMPERATURA;
        timerTerminado = false;
        displayText("Temperatura: " + String(getTemperaturaBmp()) + " C");
        //displayText("temperatura");
      }
      break;
  }
}
