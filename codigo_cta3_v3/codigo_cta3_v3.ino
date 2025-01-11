#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <SoftwareSerial.h>

// Definição da classe PID para controle de temperatura e umidade
class PID {
private:
    double kp, ki, kd;
    double setpoint;
    double integral;
    double previousError;
    double outputMin, outputMax;

public:
    PID(double kp, double ki, double kd, double outputMin = 0, double outputMax = 100) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->outputMin = outputMin;
        this->outputMax = outputMax;
        integral = 0;
        previousError = 0;
    }

    void setSetpoint(double sp) {
        setpoint = sp;
    }

    double compute(double current, double deltaTime) {
        double error = setpoint - current;
        double proportional = kp * error;
        integral += error * deltaTime;
        if (integral > outputMax) integral = outputMax;
        else if (integral < outputMin) integral = outputMin;
        double integralTerm = ki * integral;
        double derivative = kd * (error - previousError) / deltaTime;
        previousError = error;
        double output = proportional + integralTerm + derivative;
        if (output > outputMax) output = outputMax;
        else if (output < outputMin) output = outputMin;
        return output;
    }
};

// Função para calcular distância
double distancia(double duracao, int numero) {
    double dist = duracao * 340.0 / 20000.0;
    if (dist < 2 || dist > 400) {
        Serial.println("Valor fora do alcance do sensor " + String(numero));
        return 0;
    }
    return dist;
}

#define NUM_SAMPLES 10
double readPotentiometer(int pin) {
    long sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum += analogRead(pin);
        delay(10);
    }
    return sum / NUM_SAMPLES;
}

#define DHT_PIN 4
#define DHTTYPE DHT22

#define pCONTROLE_T 9
#define pCONTROLE_H 10

#define INTERVALO 2000

PID meuPid_T(1.0, 0.1, 0.01, -10);
PID meuPid_H(1.0, 0.1, 0.01, -10);

DHT dht(DHT_PIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x3f, 16, 2);

const int trigPins[] = {22, 24, 26, 28};
const int echoPins[] = {23, 25, 27, 29};

unsigned long previousMillis = 0;
double alpha = 0.6;
double lastSetTemp = 20.0;
double lastSetUmid = 30.0;

void controleRele(double output, int pinoRele, int c) {
    if (c == 1) {
        digitalWrite(pinoRele, output < 0.0 ? HIGH : LOW);
    } else {
        digitalWrite(pinoRele, output > 0.0 ? HIGH : LOW);
    }
}

void setup() {
    for (int i = 0; i < 4; i++) {
        pinMode(trigPins[i], OUTPUT);
        pinMode(echoPins[i], INPUT);
    }

    pinMode(pCONTROLE_T, OUTPUT);
    pinMode(pCONTROLE_H, OUTPUT);

    pinMode(A0, INPUT);
    pinMode(A1, INPUT);

    Serial.begin(9600);
    Serial3.begin(115200);
    dht.begin();

    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("SetTemp: ");
    lcd.setCursor(15, 0);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("SetUmid: ");
    lcd.setCursor(15, 1);
    lcd.print("%");
}

void loop() {
    for (int i = 0; i < 4; i++) {
        digitalWrite(trigPins[i], HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPins[i], LOW);

        double duracao = pulseIn(echoPins[i], HIGH);
        double distanciaSensor = distancia(duracao, i + 1);
        Serial.println("Distância do sensor " + String(i + 1) + ": " + String(distanciaSensor) + " cm");
        Serial3.print("DIST" + String(i + 1) + ":" + String(distanciaSensor) + "\n");
        delay(50);
    }

    unsigned long currentMillis = millis();
    double deltaTime = (currentMillis - previousMillis) / 1000.0;
    previousMillis = currentMillis;

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (!isnan(h) && !isnan(t)) {
        double auxTemp = readPotentiometer(A0);
        double SetTemp = map(auxTemp, 0, 1023, 20, 150);
        SetTemp = (alpha * SetTemp) + ((1 - alpha) * lastSetTemp);
        lastSetTemp = SetTemp;

        double auxHum = readPotentiometer(A1);
        double SetUmid = map(auxHum, 0, 1023, 30, 250);
        SetUmid = (alpha * SetUmid) + ((1 - alpha) * lastSetUmid);
        lastSetUmid = SetUmid;

        meuPid_T.setSetpoint(SetTemp);
        meuPid_H.setSetpoint(SetUmid);

        double outputT = meuPid_T.compute(t, deltaTime);
        double outputH = meuPid_H.compute(h, deltaTime);

        controleRele(outputT, pCONTROLE_T, 0);
        controleRele(outputH, pCONTROLE_H, 1);

        lcd.setCursor(9, 0);
        lcd.print(SetTemp, 1);
        lcd.setCursor(9, 1);
        lcd.print(SetUmid, 1);

        Serial.println("Temperatura: " + String(t));
        Serial.println("Umidade: " + String(h));
        Serial.println("SETPOINT Temperatura: " + String(SetTemp));
        Serial.println("SETPOINT Umidade: " + String(SetUmid));
        Serial.println("Controle Temperatura: " + String(outputT));
        Serial.println("Controle Umidade: " + String(outputH));
    } else {
        Serial.println("Falha na leitura do sensor DHT");
    }

    delay(INTERVALO);
}
