#include <Arduino.h>

#include <PID_v1.h>

// Pin definitions
#define HEATER_1_PIN 9
#define HEATER_2_PIN 10
#define HEATER_3_PIN 11
#define THERMISTOR_PIN A13
#define MAX_OUTPUT_PWM 110

// Thermistor parameters
#define THERMISTOR_SERIES_RESISTOR 10000 // Series resistor value in ohms
#define THERMISTOR_NOMINAL_RESISTANCE 100000 // Nominal resistance of the thermistor at 25°C
#define THERMISTOR_NOMINAL_TEMPERATURE 25 // Nominal temperature of the thermistor in °C
#define THERMISTOR_B_COEFFICIENT 3950 // Beta coefficient of the thermistor

// PID parameters
double setpoint = 120; // Target temperature in °C
double input, output;
double Kp = 50, Ki = 10, Kd = 1; // PID constants
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);


// Function to read temperature from NTC thermistor
double readTemperature() {
  int rawADC = analogRead(THERMISTOR_PIN);
  double resistance = THERMISTOR_SERIES_RESISTOR / ((1023.0 / rawADC) - 1);
  double steinhart;
  steinhart = resistance / THERMISTOR_NOMINAL_RESISTANCE; // (R/Ro)
  steinhart = log(steinhart); // ln(R/Ro)
  steinhart /= THERMISTOR_B_COEFFICIENT; // 1/B * ln(R/Ro)
  steinhart += 1.0 / (THERMISTOR_NOMINAL_TEMPERATURE + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart; // Invert
  steinhart -= 273.15; // Convert to Celsius
  steinhart += 40;      // Random offset value I Found
  return steinhart;
}


void setup() {
  // Initialize Serial communication
  Serial.begin(115200);

  // Set heater pins as output
  pinMode(HEATER_1_PIN, OUTPUT);
  pinMode(HEATER_2_PIN, OUTPUT);
  pinMode(HEATER_3_PIN, OUTPUT);

  // Initialize PID
  pid.SetMode(AUTOMATIC);

  // Set PWM frequency for heater control (for MOSFETs)
  analogWrite(HEATER_1_PIN, 0); // Set PWM frequency to approximately 980 Hz
  analogWrite(HEATER_2_PIN, 0); // Set PWM frequency to approximately 980 Hz
  analogWrite(HEATER_3_PIN, 0); // Set PWM frequency to approximately 980 Hz
}

void loop() {
  if (Serial.available() > 0) {
    String userInput = Serial.readStringUntil('\n');
    int newSetpoint = userInput.toInt();

    if (newSetpoint >= 10 && newSetpoint <= 200) {
      setpoint = newSetpoint;
      Serial.print("Setpoint updated to ");
      Serial.print(setpoint);
      Serial.println(" °C");
    } else {
      Serial.println("Please enter a value between 10 and 200 for setpoint");
    }
  }

  // Read temperature from thermistor
  double temperature = readTemperature();

  // Set the current temperature as the input for PID
  input = temperature;

  // Compute PID output (heater control signal)
  pid.Compute();

  output = map(output, 0, 255, 0, MAX_OUTPUT_PWM);

  // Apply PID output to control heaters
  analogWrite(HEATER_1_PIN, min(output, MAX_OUTPUT_PWM));
  analogWrite(HEATER_2_PIN, min(output, MAX_OUTPUT_PWM));
  analogWrite(HEATER_3_PIN, min(output, MAX_OUTPUT_PWM));

  // Print current and target temperatures
  Serial.print("Current_output: ");
  Serial.print(output);
  Serial.print(",Current_Temperature_C: ");
  Serial.print(temperature);
  Serial.print(",Target_Temperature_C: ");
  Serial.print(setpoint);
  Serial.println(" ");

  delay(1000); // Delay for stability, adjust as needed
  
}