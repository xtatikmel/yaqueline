#include <AccelStepper.h>  // Library to control stepper motors
#include <Wire.h>          // For I2C communication with the LCD display
#include <LiquidCrystal_I2C.h>  // Library to control I2C LCD

// Define motor control pins
#define dirPin 2         // Direction pin for stepper motor
#define stepPin 3        // Step pin for stepper motor
#define motorInterfaceType 1  // Driver interface type for stepper motor

#define PWM_pin 6        // PWM output pin for controlling heater or fan

// Create an instance of AccelStepper with driver type and pin setup
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

// Variables for potentiometer readings and control values
int pot, pot_p, val1 = 0, val2 = 0; 
uint16_t Dm; // Placeholder variable, could represent something specific

// Thermistor and temperature-related variables
double T0 = 25 + 273.15,  // Reference temperature in Kelvin (T0 = 25°C + 273.15 to convert to Kelvin)
       R0 = 100000,       // Reference resistance of thermistor at T0 (in ohms)
       Rb = 10000,        // Series resistor value for thermistor circuit (in ohms)
       beta = 3950;       // Beta coefficient of the thermistor (material-specific)
double Tk = 0, Tc = 0, R = 0; // Temperature variables: Tc (Celsius), Tk (Kelvin), and R (Resistance)

// PID control variables
int erreur = 0, P_erreur = 0;  // Error and previous error for PID
int T_pwm = 0, E = 0, i = 0;   // PID output, error, and intermediate control variables
float elapsedTime, Time, timePrev;  // Time tracking for PID computation

// PID constants for proportional, integral, and derivative gains
int kp = 27;   
int ki = 0.6;    
int kd = 0.6;    

// PID control variables for storing different terms
int PID_p = 0;  // Proportional term
int PID_i = 0;  // Integral term
int PID_d = 0;  // Derivative term

// Motor and temperature setpoints
int Mset = 0;   // Motor speed setpoint
int Tset = 40;  // Temperature setpoint in Celsius

// String for storing LCD data (unused here)
String data = "";

// Initialize LCD with I2C address (0x27) and 16x2 character size
LiquidCrystal_I2C lcd(0x20, 16, 2);

void setup()
{
  lcd.init();  // Initialize the LCD
  lcd.backlight();  // Turn on LCD backlight
  
  // Configure input pins
  pinMode(10, INPUT);  // Input pin for user control (e.g., motor mode)
  pinMode(11, INPUT);  // Input pin for user control (e.g., temperature mode)
  
  // Configure PWM pin as output
  pinMode(PWM_pin, OUTPUT);
  
  // Set the maximum speed for the stepper motor
  stepper.setMaxSpeed(1000);
  
  // Initialize time for PID calculations
  Time = millis();
  
  // Display an initial message on the LCD
  lcd.setCursor(0, 0);
  lcd.print("  Config.T&S ");
}

void loop()
{
  // Read current temperature from thermistor
  Tc = thermistor();
  
  // Read potentiometer input value (analog)
  pot = analogRead(A0);
  
  // Read digital inputs from control buttons (pin 10 and 11)
  val1 = digitalRead(10);  // First button for controlling motor
  val2 = digitalRead(11);  // Second button for controlling temperature

  // Control mode 1: Motor speed control
  if (val1 == 1) {
    i = 1;  // Set control flag
    stepper.stop();  // Stop the motor when changing speed
    
    // Map the potentiometer value to a range for motor speed
    pot = map(pot, 0, 1023, 10, 1000);  
    
    // Display motor speed on LCD
    lcd.setCursor(0, 0);
    lcd.print("Motor Speed:      ");
    
    if (pot != pot_p) {  // Only update if the potentiometer value changed
      lcd.setCursor(0, 1);
      lcd.print("(R.12)            ");
      lcd.setCursor(9, 1);
      lcd.print(map(pot, 10, 1000, 1, 100));  // Display percentage
      lcd.setCursor(12, 1);
      lcd.print("%");
      
      // Set motor speed
      Mset = pot;
      pot_p = pot;  // Store previous potentiometer value
    }
  }
  // Control mode 2: Temperature setpoint control
  else if (val2 == 1) {
    i = 1;  // Set control flag
    stepper.stop();  // Stop the motor
    
    // Map the potentiometer to a temperature range
    pot = map(pot, 0, 1023, 20, 300);  
    
    // Display the set temperature on the LCD
    lcd.setCursor(0, 0);
    lcd.print("Val temperature:      ");
    
    if (pot != pot_p) {  // Update only if the potentiometer changed
      lcd.setCursor(0, 1);
      lcd.print("(R.260)                    ");
      lcd.setCursor(9, 1);
      lcd.print(pot);  // Display set temperature
      lcd.setCursor(12, 1);
      lcd.print(" C");
      pot_p = pot;
      
      // Set the temperature setpoint
      Tset = pot;
    }
  }
  // Normal operation mode: Run motor and control temperature
  else {
    if (i == 1) {
      i = 0;  // Reset control flag
      lcd.setCursor(0, 0);
      lcd.print("Temperature :      ");
      lcd.print("                   ");
      lcd.setCursor(0, 1);
      lcd.print("                   ");
      lcd.setCursor(8, 1);
      lcd.print(int(Tc));  // Display current temperature
    }
    
    // Set the stepper motor speed
    stepper.setSpeed(Mset);
    
    // Update the display periodically
    updateDisplay();
    
    // Run the stepper motor at the set speed
    stepper.runSpeed();
    
    // Control heater/fan using PWM based on temperature
    analogWrite(PWM_pin, T_pwm);
    P_erreur = E;  // Store previous error
  }
  
  // PID control for temperature
  erreur = (Tset * 1.034) - Tc;  // Calculate error (setpoint - current temperature)
  P_erreur = 0;
  
  // Prevent negative errors (heating is only needed when below the setpoint)
  if (erreur <= 0) {
    erreur = 0;
  }
  
  // Map the error to a PWM value
  E = map(erreur, 0, int(Tc), 0, 255);
  
  // Proportional term
  PID_p = E * kp;
  
  // Integral term (only accumulate when error is small)
  if (-3 < E && E < 3) {
    PID_i = PID_i + (ki * E);
  }
  
  // Calculate elapsed time for derivative term
  timePrev = Time;
  Time = millis();
  elapsedTime = (Time - timePrev) / 1000;
  
  // Derivative term
  PID_d = kd * ((E - P_erreur) / elapsedTime);
  
  // Calculate total PWM output from PID terms
  T_pwm = PID_p + PID_i + PID_d;
  
  // Limit PWM to valid range (0-255)
  if (T_pwm < 0) {
    T_pwm = 0;
  } else if (T_pwm > 255) {
    T_pwm = 255;
  }
  
  // Output the final PWM value
  analogWrite(PWM_pin, T_pwm);
  P_erreur = E;  // Store previous error for the next loop
}

// Function to calculate temperature using a thermistor
double thermistor()
{
  int t = analogRead(A2);  // Read analog value from thermistor
  float tr = 1023.0 / t - 1;  // Convert analog reading to resistance
  tr = Rb / tr;  // Calculate thermistor resistance
  
  // Apply the Steinhart-Hart equation to calculate temperature
  float steinhart;
  steinhart = tr / R0;
  steinhart = log(steinhart);
  steinhart /= beta;
  steinhart += 1.0 / (T0);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;  // Convert temperature back to Celsius
  
  return steinhart;  // Return the calculated temperature
}

// Function to periodically update the LCD display with temperature
void updateDisplay()
{
  static unsigned long timer = 0;  // Track time for updates
  unsigned long interval = 3000;   // Update display every 3 seconds
  
  if (millis() - timer >= interval) {
    timer = millis();  // Reset the timer
    lcd.print("      ");  // Clear the previous temperature
    lcd.setCursor(8, 1);  // Move cursor to display position
    lcd.print(Tc);  // Display the current temperature
  }
}
