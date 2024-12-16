#include <DHT.h>
#include <LiquidCrystal.h>

// Pin Definitions and Registers
#define START_BUTTON 2
#define RESET_BUTTON 3
#define FAN_PIN 9
#define DHT_PIN 23
#define DHT_TYPE DHT11
#define YELLOW_LED 50
#define GREEN_LED 52
#define RED_LED 46
#define BLUE_LED 48
#define LCD_RS 31
#define LCD_EN 33
#define LCD_D4 35
#define LCD_D5 37
#define LCD_D6 39
#define LCD_D7 41
#define STEPPER_IN1 8
#define STEPPER_IN2 10
#define STEPPER_IN3 7
#define STEPPER_IN4 11
#define WATER_BUTTON 22
#define STEPPER_BUTTON 18
#define TEMP_THRESHOLD 80
#define UPDATE_INTERVAL 60000

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
DHT dht(DHT_PIN, DHT_TYPE);

enum States { DISABLED, IDLE, ERROR, RUNNING };
volatile States currentState = DISABLED;
volatile bool startButtonPressed = false;
volatile bool resetButtonPressed = false;
volatile bool stepperButtonPressed = false;
unsigned long lastUpdateMillis = 0;

void configurePinAsInputPullup(uint8_t pin) {
  if (pin < 8) { // Pins 0-7 on PORTD
    DDRD &= ~(1 << pin);      // Set pin as input
    PORTD |= (1 << pin);      // Enable pull-up resistor
  } else if (pin < 14) { // Pins 8-13 on PORTB
    DDRB &= ~(1 << (pin - 8)); // Set pin as input
    PORTB |= (1 << (pin - 8)); // Enable pull-up resistor
  } else { // Pins 14 and above on PORTC
    DDRC &= ~(1 << (pin - 14)); // Set pin as input
    PORTC |= (1 << (pin - 14)); // Enable pull-up resistor
  }
}

// Function to configure pins
void configurePinAsInput(uint8_t pin) {
  if (pin < 8) {
    DDRD &= ~(1 << pin);
  } else if (pin < 14) {
    DDRB &= ~(1 << (pin - 8));
  } else {
    DDRC &= ~(1 << (pin - 14));
  }
}

void configurePinAsOutput(uint8_t pin) {
  if (pin < 8) {
    DDRD |= (1 << pin);
  } else if (pin < 14) {
    DDRB |= (1 << (pin - 8));
  } else {
    DDRC |= (1 << (pin - 14));
  }
}

void writePin(uint8_t pin, bool value) {
  if (pin < 8) {
    if (value) PORTD |= (1 << pin);
    else PORTD &= ~(1 << pin);
  } else if (pin < 14) {
    if (value) PORTB |= (1 << (pin - 8));
    else PORTB &= ~(1 << (pin - 8));
  } else {
    if (value) PORTC |= (1 << (pin - 14));
    else PORTC &= ~(1 << (pin - 14));
  }
}

bool readPin(uint8_t pin) {
  if (pin < 8) {
    return PIND & (1 << pin);
  } else if (pin < 14) {
    return PINB & (1 << (pin - 8));
  } else {
    return PINC & (1 << (pin - 14));
  }
}

// Interrupt Handlers
void handleStartButton() {
  startButtonPressed = true;
}

void handleResetButton() {
  resetButtonPressed = true;
}

void handleStepperButton() {
  stepperButtonPressed = true;
}

void setup() {
  Serial.begin(9600);

  // Configure pins
  configurePinAsInputPullup(START_BUTTON);
  configurePinAsInputPullup(RESET_BUTTON);
  configurePinAsInputPullup(WATER_BUTTON);
  configurePinAsInputPullup(STEPPER_BUTTON);
  
  configurePinAsOutput(FAN_PIN);
  configurePinAsOutput(YELLOW_LED);
  configurePinAsOutput(GREEN_LED);
  configurePinAsOutput(RED_LED);
  configurePinAsOutput(BLUE_LED);
  configurePinAsOutput(STEPPER_IN1);
  configurePinAsOutput(STEPPER_IN2);
  configurePinAsOutput(STEPPER_IN3);
  configurePinAsOutput(STEPPER_IN4);

  dht.begin(); // Initialize DHT sensor
  lcd.begin(16, 2); // Initialize LCD

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(START_BUTTON), handleStartButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(RESET_BUTTON), handleResetButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(STEPPER_BUTTON), handleStepperButton, FALLING);
}

void loop() {
  switch (currentState) {
    case DISABLED:
      handleDisabledMode();
      break;
    case IDLE:
      handleIdleMode();
      break;
    case ERROR:
      handleErrorMode();
      break;
    case RUNNING:
      handleRunningMode();
      break;
  }
  updateDisplay();
  handleStepperMotor();
}

void handleDisabledMode() {
  setLEDState(YELLOW_LED);
  writePin(FAN_PIN, LOW);

  if (startButtonPressed) {
    startButtonPressed = false;
    logStateTransition("DISABLED", "IDLE");
    currentState = IDLE;
  }
}

void handleIdleMode() {
  setLEDState(GREEN_LED);
  writePin(FAN_PIN, LOW);

  if (startButtonPressed) {
    startButtonPressed = false;
    logStateTransition("IDLE", "DISABLED");
    currentState = DISABLED;
  }

  float temperatureC = dht.readTemperature();
  float temperatureF = temperatureC * 9.0 / 5.0 + 32.0;

  if (temperatureF >= TEMP_THRESHOLD) {
    logStateTransition("IDLE", "RUNNING");
    currentState = RUNNING;
    return;
  }

  if (!readPin(WATER_BUTTON)) {
    logStateTransition("IDLE", "ERROR");
    currentState = ERROR;
  }
}

void handleErrorMode() {
  setLEDState(RED_LED);
  writePin(FAN_PIN, LOW);
  lcd.clear();
  lcd.print("Water level too");
  lcd.setCursor(0, 1);
  lcd.print("low");

  if (resetButtonPressed) {
    resetButtonPressed = false;
    logStateTransition("ERROR", "IDLE");
    currentState = IDLE;
    lcd.clear();
  } else if (startButtonPressed) {
    startButtonPressed = false;
    logStateTransition("ERROR", "DISABLED");
    currentState = DISABLED;
    lcd.clear();
  }
}

void handleRunningMode() {
  setLEDState(BLUE_LED);
  writePin(FAN_PIN, HIGH);

  float temperatureC = dht.readTemperature();
  float temperatureF = temperatureC * 9.0 / 5.0 + 32.0;

  if (temperatureF < TEMP_THRESHOLD) {
    writePin(FAN_PIN, LOW);
    logStateTransition("RUNNING", "IDLE");
    currentState = IDLE;
    return;
  }

  if (!readPin(WATER_BUTTON)) {
    writePin(FAN_PIN, LOW);
    logStateTransition("RUNNING", "ERROR");
    currentState = ERROR;
    return;
  }

  if (startButtonPressed) {
    startButtonPressed = false;
    writePin(FAN_PIN, LOW);
    logStateTransition("RUNNING", "DISABLED");
    currentState = DISABLED;
  }
}

void setLEDState(int activeLED) {
  writePin(YELLOW_LED, activeLED == YELLOW_LED);
  writePin(GREEN_LED, activeLED == GREEN_LED);
  writePin(RED_LED, activeLED == RED_LED);
  writePin(BLUE_LED, activeLED == BLUE_LED);
}

void updateDisplay() {
  if (currentState == ERROR) return;

  unsigned long currentMillis = millis();
  if (currentMillis - lastUpdateMillis >= UPDATE_INTERVAL) {
    lastUpdateMillis = currentMillis;

    float temperatureC = dht.readTemperature();
    float temperatureF = temperatureC * 9.0 / 5.0 + 32.0;
    float humidity = dht.readHumidity();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temperatureF);
    lcd.print(" F");

    lcd.setCursor(0, 1);
    lcd.print("Hum: ");
    lcd.print(humidity);
    lcd.print(" %");
  }
}

void logStateTransition(const char* fromState, const char* toState) {
  unsigned long currentMillis = millis();
  Serial.print("Transition: ");
  Serial.print(fromState);
  Serial.print(" -> ");
  Serial.print(toState);
  Serial.print(" at time: ");
  Serial.print(currentMillis / 1000);
  Serial.println(" seconds");
}

void handleStepperMotor() {
  if (stepperButtonPressed && currentState != ERROR) {
    stepperButtonPressed = false;
    for (int i = 0; i < 10; i++) {
      writePin(STEPPER_IN1, HIGH);
      writePin(STEPPER_IN2, LOW);
      writePin(STEPPER_IN3, LOW);
      writePin(STEPPER_IN4, LOW);
      delay(10);

      writePin(STEPPER_IN1, LOW);
      writePin(STEPPER_IN2, HIGH);
      writePin(STEPPER_IN3, LOW);
      writePin(STEPPER_IN4, LOW);
      delay(10);

      writePin(STEPPER_IN1, LOW);
      writePin(STEPPER_IN2, LOW);
      writePin(STEPPER_IN3, HIGH);
      writePin(STEPPER_IN4, LOW);
      delay(10);

      writePin(STEPPER_IN1, LOW);
      writePin(STEPPER_IN2, LOW);
      writePin(STEPPER_IN3, LOW);
      writePin(STEPPER_IN4, HIGH);
      delay(10);
    }
  }
}
