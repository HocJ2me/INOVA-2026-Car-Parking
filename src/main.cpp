#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

constexpr uint8_t STATION_COUNT = 3;

constexpr uint8_t DHT_PIN = 7;
constexpr uint8_t BUZZER_PIN = 8;
constexpr uint8_t SOLAR_PIN = A3;

constexpr int GRID_POWER_W = 600;
constexpr int SOLAR_MAX_W = 400;
constexpr int SYSTEM_MAX_W = 1000;
constexpr int STATION_MAX_W = 500;
constexpr float MAX_TEMPERATURE_C = 50.0;

LiquidCrystal_I2C lcd(0x27, 20, 4);
DHT dht(DHT_PIN, DHT22);

struct ChargingStation {
  uint8_t socPin;
  uint8_t buttonPin;
  uint8_t powerLedPin;

  bool connected;
  bool lastRawButton;
  bool stableButton;

  unsigned long debounceTime;

  int soc;
  int powerW;
};

ChargingStation stations[STATION_COUNT] = {
  {A0, 2, 9, true, HIGH, HIGH, 0, 0, 0},
  {A1, 3, 10, true, HIGH, HIGH, 0, 0, 0},
  {A2, 4, 11, true, HIGH, HIGH, 0, 0, 0}
};

int solarPowerW = 0;
int availablePowerW = 0;
float batteryTemperature = 25.0;
bool temperatureAlarm = false;

unsigned long lastControlTime = 0;
unsigned long lastDisplayTime = 0;
unsigned long lastTemperatureTime = 0;
unsigned long lastSerialTime = 0;

void printLcdLine(uint8_t row, const char *text) {
  lcd.setCursor(0, row);

  uint8_t length = strlen(text);

  for (uint8_t i = 0; i < 20; i++) {
    if (i < length) {
      lcd.print(text[i]);
    } else {
      lcd.print(' ');
    }
  }
}

void updateButtons() {
  unsigned long now = millis();

  for (uint8_t i = 0; i < STATION_COUNT; i++) {
    bool reading = digitalRead(stations[i].buttonPin);

    if (reading != stations[i].lastRawButton) {
      stations[i].debounceTime = now;
      stations[i].lastRawButton = reading;
    }

    if (now - stations[i].debounceTime >= 30) {
      if (reading != stations[i].stableButton) {
        stations[i].stableButton = reading;

        if (reading == LOW) {
          stations[i].connected = !stations[i].connected;
        }
      }
    }
  }
}

void readSocValues() {
  for (uint8_t i = 0; i < STATION_COUNT; i++) {
    int rawValue = analogRead(stations[i].socPin);
    stations[i].soc = map(rawValue, 0, 1023, 0, 100);
  }
}

void readSolarPower() {
  int rawLight = analogRead(SOLAR_PIN);

  // Cảm biến LDR của Wokwi có giá trị analog thấp hơn khi ánh sáng mạnh.
  solarPowerW = map(rawLight, 1023, 0, 0, SOLAR_MAX_W);
  solarPowerW = constrain(solarPowerW, 0, SOLAR_MAX_W);

  availablePowerW = GRID_POWER_W + solarPowerW;
  availablePowerW = constrain(
    availablePowerW,
    0,
    SYSTEM_MAX_W
  );
}

void readTemperature() {
  if (millis() - lastTemperatureTime < 2000) {
    return;
  }

  lastTemperatureTime = millis();

  float newTemperature = dht.readTemperature();

  if (!isnan(newTemperature)) {
    batteryTemperature = newTemperature;
  }

  temperatureAlarm =
    batteryTemperature >= MAX_TEMPERATURE_C;
}

bool canCharge(uint8_t index) {
  return stations[index].connected &&
         stations[index].soc < 95 &&
         !temperatureAlarm;
}

int priorityWeight(uint8_t index) {
  // Xe có SOC thấp sẽ được ưu tiên công suất cao hơn.
  return 101 - stations[index].soc;
}

void allocateChargingPower() {
  for (uint8_t i = 0; i < STATION_COUNT; i++) {
    stations[i].powerW = 0;
  }

  if (temperatureAlarm) {
    return;
  }

  int remainingPower = availablePowerW;

  // Chạy nhiều vòng để phân phối lại phần công suất thừa
  // khi một trạm đã đạt giới hạn 500 W.
  for (uint8_t pass = 0;
       pass < STATION_COUNT && remainingPower > 0;
       pass++) {

    long totalWeight = 0;

    for (uint8_t i = 0; i < STATION_COUNT; i++) {
      if (canCharge(i) &&
          stations[i].powerW < STATION_MAX_W) {
        totalWeight += priorityWeight(i);
      }
    }

    if (totalWeight == 0) {
      break;
    }

    int powerAtStart = remainingPower;
    int assignedThisPass = 0;

    for (uint8_t i = 0; i < STATION_COUNT; i++) {
      if (!canCharge(i) ||
          stations[i].powerW >= STATION_MAX_W) {
        continue;
      }

      int weight = priorityWeight(i);

      int share = static_cast<long>(powerAtStart) *
                  weight / totalWeight;

      if (share < 1) {
        share = 1;
      }

      int headroom =
        STATION_MAX_W - stations[i].powerW;

      int allocated = min(share, headroom);
      allocated = min(allocated, remainingPower);

      stations[i].powerW += allocated;
      remainingPower -= allocated;
      assignedThisPass += allocated;
    }

    if (assignedThisPass == 0) {
      break;
    }
  }
}

void updatePowerIndicators() {
  for (uint8_t i = 0; i < STATION_COUNT; i++) {
    int brightness = map(
      stations[i].powerW,
      0,
      STATION_MAX_W,
      0,
      255
    );

    brightness = constrain(brightness, 0, 255);
    analogWrite(stations[i].powerLedPin, brightness);
  }

  if (temperatureAlarm) {
    tone(BUZZER_PIN, 1200);
  } else {
    noTone(BUZZER_PIN);
  }
}

void updateLcd() {
  if (millis() - lastDisplayTime < 500) {
    return;
  }

  lastDisplayTime = millis();

  char line[21];

  if (temperatureAlarm) {
    snprintf(
      line,
      sizeof(line),
      "ALARM TEMP:%2dC",
      static_cast<int>(batteryTemperature)
    );
  } else {
    snprintf(
      line,
      sizeof(line),
      "PV:%3dW CAP:%4dW",
      solarPowerW,
      availablePowerW
    );
  }

  printLcdLine(0, line);

  for (uint8_t i = 0; i < STATION_COUNT; i++) {
    const char *state;

    if (temperatureAlarm) {
      state = "HOT";
    } else if (!stations[i].connected) {
      state = "OFF";
    } else if (stations[i].soc >= 95) {
      state = "FUL";
    } else {
      state = "ON ";
    }

    snprintf(
      line,
      sizeof(line),
      "S%d %s %3d%% %3dW",
      i + 1,
      state,
      stations[i].soc,
      stations[i].powerW
    );

    printLcdLine(i + 1, line);
  }
}

void printSerialData() {
  if (millis() - lastSerialTime < 1000) {
    return;
  }

  lastSerialTime = millis();

  Serial.print(F("PV="));
  Serial.print(solarPowerW);

  Serial.print(F("W, CAP="));
  Serial.print(availablePowerW);

  Serial.print(F("W, TEMP="));
  Serial.print(batteryTemperature, 1);
  Serial.print(F("C"));

  for (uint8_t i = 0; i < STATION_COUNT; i++) {
    Serial.print(F(" | S"));
    Serial.print(i + 1);

    Serial.print(F(": SOC="));
    Serial.print(stations[i].soc);

    Serial.print(F("%, P="));
    Serial.print(stations[i].powerW);

    Serial.print(F("W, "));
    Serial.print(stations[i].connected ? F("ON") : F("OFF"));
  }

  Serial.println();
}

void runEnergyController() {
  if (millis() - lastControlTime < 200) {
    return;
  }

  lastControlTime = millis();

  readSocValues();
  readSolarPower();
  allocateChargingPower();
  updatePowerIndicators();
}

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < STATION_COUNT; i++) {
    pinMode(stations[i].buttonPin, INPUT_PULLUP);
    pinMode(stations[i].powerLedPin, OUTPUT);
  }

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SOLAR_PIN, INPUT);

  dht.begin();

  lcd.init();
  lcd.backlight();
  lcd.clear();

  printLcdLine(0, "SMART EV CHARGING");
  printLcdLine(1, "3 STATIONS + AI");
  printLcdLine(2, "ENERGY CONTROLLER");
  printLcdLine(3, "INITIALIZING...");

  delay(1500);
  lcd.clear();
}

void loop() {
  updateButtons();
  readTemperature();
  runEnergyController();
  updateLcd();
  printSerialData();
}