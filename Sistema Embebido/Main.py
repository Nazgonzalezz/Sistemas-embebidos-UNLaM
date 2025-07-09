#include <LiquidCrystal.h>
#include <DHT.h>

#define DHTPIN 2
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

#define stepPin 8
#define dirPin 9

void setup() {
  lcd.begin(20, 4);
  dht.begin();

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  lcd.print("Iniciando sistema...");
  delay(2000);
  lcd.clear();
}

void loop() {
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  int joyX = analogRead(A0);
  int joyY = analogRead(A1);

  lcd.setCursor(0, 0);
  lcd.print("Temp: "); lcd.print(temp); lcd.print(" C");
  lcd.setCursor(0, 1);
  lcd.print("Hum: "); lcd.print(hum); lcd.print(" %");
  lcd.setCursor(0, 2);
  lcd.print("Joy X: "); lcd.print(joyX);
  lcd.setCursor(0, 3);
  lcd.print("Joy Y: "); lcd.print(joyY);

  if (temp > 28 || joyY > 800) {
    digitalWrite(dirPin, HIGH);
    for (int i = 0; i < 200; i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
    }
  }

  delay(1000);
}
