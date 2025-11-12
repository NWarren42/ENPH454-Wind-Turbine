// MXUS XF40 Hall RPM reader — Arduino Mega2560
// Pins (your wiring):
//   Blue  hall -> D38
//   Green hall -> D34
//   Yellow hall -> D30
//
// Assumptions:
//   - 23 pole pairs  => 6 hall edges / electrical cycle => 138 edges / mechanical rev
//   - Hall outputs are open-collector with external 3.3V pull-ups
//   - Motor GND tied to Arduino GND

#include <Arduino.h>

const uint8_t HALL_BLUE_PIN  = 38;
const uint8_t HALL_GREEN_PIN = 34;
const uint8_t HALL_YELLOW_PIN= 30;
const uint8_t VOLTAGE_PIN    = A0;

const uint8_t POLE_PAIRS = 23;
const uint16_t EDGES_PER_REV = 6 * POLE_PAIRS; // 138 for 23 pp. 6 because rising and falling edges on 3 halls

// Analog sensing (High Resistor: Low Resistor divider -> A0)
const float HIGH_RESISTOR_kR = 220.0f; // kOhm
const float LOW_RESISTOR_kR  = 10.0f;   // kOhm
const float ADC_REF_V = 5.0f;
const float VOLTAGE_DIVIDER_GAIN = (HIGH_RESISTOR_kR + LOW_RESISTOR_kR) / LOW_RESISTOR_kR; // multiply ADC voltage back to line voltage
const float ADC_V_PER_COUNT = ADC_REF_V / 1023.0f;

// Sampling / reporting
const uint32_t SAMPLE_HZ = 10000UL;   // 10 kHz sampler (Timer1 ISR)
const uint32_t REPORT_MS = 200;       // print every X ms
const float    ALPHA = 0.3f;          // EMA smoothing for RPM printout

// Volatile state touched in ISR
volatile uint8_t lastA = 0, lastB = 0, lastC = 0;
volatile uint32_t edgeCount = 0;

void setupTimer1_10kHz() {
  // Timer1 CTC @ 10 kHz on 16 MHz clock
  // OCR1A = (F_CPU / (prescaler * target)) - 1
  // Use prescaler 8: OCR1A = 16e6 / (8*10e3) - 1 = 199
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A  = 199;
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS11);    // prescaler 8
  TIMSK1 |= (1 << OCIE1A);  // enable compare match A interrupt
  interrupts();
}

void setup() {
  Serial.begin(115200);

  pinMode(HALL_BLUE_PIN,   INPUT); // external pull-ups -> keep INPUT (no pullup)
  pinMode(HALL_GREEN_PIN,  INPUT);
  pinMode(HALL_YELLOW_PIN, INPUT);
  pinMode(VOLTAGE_PIN,     INPUT);

  // Initialize last states
  lastA = digitalRead(HALL_BLUE_PIN);
  lastB = digitalRead(HALL_GREEN_PIN);
  lastC = digitalRead(HALL_YELLOW_PIN);

  analogRead(VOLTAGE_PIN); // throw away the first sample after switching mux

  setupTimer1_10kHz();

  Serial.println(F("Hall RPM monitor (Mega2560)"));
  Serial.println(F("Pins: Blue->38, Green->34, Yellow->30"));
  Serial.print(F("Edges/rev = ")); Serial.println(EDGES_PER_REV);
}

ISR(TIMER1_COMPA_vect) {
  // Sample pins (cheap enough at 10 kHz even with digitalRead)
  uint8_t a = digitalRead(HALL_BLUE_PIN);
  uint8_t b = digitalRead(HALL_GREEN_PIN);
  uint8_t c = digitalRead(HALL_YELLOW_PIN);

  // Count rising edges (0->1) on any hall line
  if (a && !lastA) edgeCount++;
  if (b && !lastB) edgeCount++;
  if (c && !lastC) edgeCount++;

  lastA = a; lastB = b; lastC = c;
}

void loop() {
  static uint32_t lastPrint = 0;
  static uint32_t lastEdgeCount = 0;
  static float rpmEMA = 0.0f;
  static float voltageEMA = 0.0f;

  uint32_t now = millis();
  if (now - lastPrint >= REPORT_MS) {
    noInterrupts();
    uint32_t edges = edgeCount;
    interrupts();

    uint32_t deltaEdges = edges - lastEdgeCount;
    lastEdgeCount = edges;

    // edges in this window -> edges per second
    float window_s = (now - lastPrint) / 1000.0f;
    lastPrint = now;

    float edges_per_s = deltaEdges / window_s;

    // RPM = (edges/s) * 60 / EDGES_PER_REV
    float rpm = edges_per_s * 60.0f / (float)EDGES_PER_REV;

    // Exponential moving average for steadier printout
    rpmEMA = (ALPHA * rpm) + (1.0f - ALPHA) * rpmEMA;

    int rawAdc = analogRead(VOLTAGE_PIN);
    float sensedVoltage = rawAdc * ADC_V_PER_COUNT * VOLTAGE_DIVIDER_GAIN;
    voltageEMA = (ALPHA * sensedVoltage) + (1.0f - ALPHA) * voltageEMA;

    Serial.print(F("RPM: "));
    Serial.print(rpm, 2);
    Serial.print(F(" Vout: "));
    Serial.print(sensedVoltage, 2);
    Serial.println(F(""));
  }
}
