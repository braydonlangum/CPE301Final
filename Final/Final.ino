//Authors: Qizhi Ge, Braydon Langum, Corey Markwardt-Abrigo

#include <LiquidCrystal.h> 
#include <Servo.h>

// --- Pin Definitions ---
const int servoPin = 12;       
const int lcdRS = 23;
const int lcdRW = 22;
const int lcdE  = 25; 
const int lcdD4 = 27;
const int lcdD5 = 29;
const int lcdD6 = 31;
const int lcdD7 = 35; 

// --- Thresholds & Timers ---
int noiseThreshold = 330;      
unsigned long activeStartTime = 0;
const unsigned long activeDuration = 60000; 

// 1-Minute Display/Log Timer
unsigned long lastLogTime = 0;
const unsigned long logInterval = 60000;

// --- Objects ---
LiquidCrystal lcd(lcdRS, lcdRW, lcdE, lcdD4, lcdD5, lcdD6, lcdD7); 
Servo actionServo;

// --- State Machine ---
enum SystemState { OFF, IDLE, ACTIVE, ERROR };
SystemState currentState;
volatile bool startTriggered = false; 

// ==========================================
// --- BARE METAL USART (NO SERIAL LIBRARY) ---
// ==========================================
void setupUSART() {
  unsigned int ubrr = 103; // 9600 baud rate for 16MHz clock
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  UCSR0B = (1 << TXEN0); // Enable transmitter
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8-bit data format
}

void usartTransmit(unsigned char data) {
  while (!(UCSR0A & (1 << UDRE0))); // Wait for empty transmit buffer
  UDR0 = data; // Put data into buffer, sends the data
}

void usartPrint(const char* str) {
  while (*str) {
    usartTransmit(*str++);
  }
}

void usartPrintNum(long num) {
  char buf[10];
  ltoa(num, buf, 10);
  usartPrint(buf);
}
// ==========================================

// --- ISR ---
void startISR() {
  if (currentState == OFF) {
    startTriggered = true;
  }
}

// --- Custom Delay ---
void customDelay(unsigned long waitTime) {
  unsigned long start = millis();
  while (millis() - start < waitTime) {}
}

// --- Custom ADC ---
void setupADC() {
  ADMUX = (1 << REFS0);
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

int readSensor() {
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  return ADC;
}

void setup() {
  setupUSART();

  DDRB |= (1 << 7) | (1 << 4) | (1 << 5); 
  DDRH |= (1 << 6) | (1 << 5); 
  DDRC |= (1 << 4); 

  DDRE &= ~(1 << 4); 
  DDRH &= ~(1 << 3) | ~(1 << 4); 

  PORTC |= (1 << 4); 

  setupADC();
  attachInterrupt(digitalPinToInterrupt(2), startISR, RISING);

  actionServo.attach(servoPin);
  actionServo.write(0); 

  lcd.begin(16, 2);
  changeState(OFF); 
}

void loop() {
  // --- 1. THE ERROR TRAP ---
  if (currentState == ERROR) {
    bool resetPressed = (PINH & (1 << 4)); 
    if (resetPressed) {
      changeState(IDLE); 
      customDelay(200);  
    } else {
      return; 
    }
  }

  // --- 2. Read Inputs ---
  bool offPressed = (PINH & (1 << 3)); 
  int sensorValue = readSensor();      

  // --- 3. Sensor Error Checking ---
  if (currentState != OFF) {
    if (sensorValue <= 5 || sensorValue >= 1020) {
      changeState(ERROR);
      return; 
    }
  }

  // --- 4. Global OFF Override ---
  if (offPressed && currentState != OFF) {
    changeState(OFF);
    customDelay(200); 
  }

  // --- 5. 1-Minute Display & Logging Routine ---
  if ((currentState == IDLE || currentState == ACTIVE) && (millis() - lastLogTime >= logInterval)) {
    lastLogTime = millis();
    
    // Update LCD
    lcd.setCursor(0, 1);
    lcd.print("Sens Val: ");
    lcd.print(sensorValue);
    lcd.print("   "); // Clear trailing characters

    // Log to Serial Monitor via Bare-Metal USART
    unsigned long totalSeconds = millis() / 1000;
    int hrs = totalSeconds / 3600;
    int mins = (totalSeconds % 3600) / 60;
    int secs = totalSeconds % 60;

    usartPrint("[RTC] Time: ");
    usartPrintNum(hrs); usartPrint(":");
    usartPrintNum(mins); usartPrint(":");
    usartPrintNum(secs);
    usartPrint(" | Sensor: ");
    usartPrintNum(sensorValue);
    usartPrint("\r\n");
  }

  // --- 6. State Machine Logic ---
  switch (currentState) {
    case OFF:
      if (startTriggered) {
        startTriggered = false; 
        changeState(IDLE);
        customDelay(200); 
      }
      break;

    case IDLE:
      startTriggered = false; 
      if (sensorValue > noiseThreshold) {
        changeState(ACTIVE);
      }
      break;

    case ACTIVE:
      startTriggered = false; 
      if (millis() - activeStartTime >= activeDuration) {
        actionServo.write(0);
        PORTB &= ~(1 << 7); 
        changeState(IDLE);
      } else {
        actionServo.write(180);        
        PORTB |= (1 << 7); 
      }
      break;
      
    case ERROR:
      break;
  }
  
  customDelay(50); 
}

// --- State Transition Helper ---
void changeState(SystemState newState) {
  currentState = newState;
  
  PORTH &= ~(1 << 6); 
  PORTB &= ~(1 << 4); 
  PORTH &= ~(1 << 5); 
  PORTB &= ~(1 << 5); 
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lastLogTime = millis(); // Reset log timer on state change

  switch (currentState) {
    case OFF:
      actionServo.write(0);         
      PORTB &= ~(1 << 7); 
      PORTH |= (1 << 6);  
      lcd.print("System OFF");
      break;
      
    case IDLE:
      PORTB |= (1 << 4);  
      lcd.print("State: IDLE");
      break;
      
    case ACTIVE:
      PORTH |= (1 << 5);  
      lcd.print("State: ACTIVE!");
      activeStartTime = millis(); 
      break;
      
    case ERROR:
      actionServo.write(0);         
      PORTB &= ~(1 << 7); 
      PORTB |= (1 << 5);  
      lcd.print("SYS ERROR");
      lcd.setCursor(0, 1);
      lcd.print("Press RESET btn");
      break;
  }
}