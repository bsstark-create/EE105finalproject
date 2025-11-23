#include <Arduino.h>
#include "AD5593R.h"
#include <Wire.h>
#include "MAX86916_eda.h"
#include <math.h>

// Module pins
#define m1_na A0
#define m1_vd A1
#define m2_diode A2
#define m2_nmos A3
#define m3_pd A6
#define module_selector A7

// DAC pins
#define dac_diode 0
#define dac_nmos_gate 1
#define dac_nmos_drain 2
#define dac_pd_pair1 3
#define dac_pd_pair2 4

// DAC/ADC values
#define dac_0v 0
#define dac_1p8v 1150
#define dac_3p3v 2000
#define adc_conversion 0.00322
#define dac_conversion 0.00165

#define PERIOD 3
#define THRESHOLD 200
#define ref_voltage 3.3

// Use built-in LED for heartbeat
#define HEART_LED_PIN LED_BUILTIN

MAX86916_eda ppg;

// Commands
enum Command {
  M1_NETWORK_ARRAY,
  M1_VOLTAGE_DIVIDER,
  M2_DIODE_MODULE,
  M2_DIODE_RAW,
  M2_TRAN_MODULE_VDS,
  M2_TRAN_MODULE_VGS,
  M2_TRAN_RAW,
  M3_LED_FLASH,
  M3_PPG,
  M3_COLLECT_CURRENT,
  M3_STEP_LED,
  LIFI_TX,
  LIFI_RX,
  EXERCISE_CLASSIFY,
  HEARTBEAT_LED,
  UNKNOWN_COMMAND,
  EXIT_COMMAND
};

void dac_setup();
void reset_dac();
Command mapStringToCommand(String command);
bool exit_condition();
void m1_network_array();
void m1_voltagedivider();
void m2_diode_module();
void m2_diode_module_raw();
void m2_tran_module_vds();
void m2_tran_module_vgs();
void m2_tran_module_raw();
void m3_led_flash();
void m3_collect_current();
void m3_step_led_brightness();
void m3_ppg();
bool read_pd();
void lifi_tx();
void send_byte(char my_byte);
void lifi_rx();
char get_byte();
void print_byte(char my_byte);
void detect_exercise_type();
void heartbeat_led();

// ----------------------------------------------------

void setup() {
  Wire.begin();
  Serial.begin(115200);
  dac_setup();
  reset_dac();
}

// ----------------------------------------------------

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();     // remove \r, spaces just in case
    Command cmd = mapStringToCommand(command);

    switch (cmd) {
      case M1_NETWORK_ARRAY:     m1_network_array(); break;
      case M1_VOLTAGE_DIVIDER:   m1_voltagedivider(); break;
      case M2_DIODE_MODULE:      m2_diode_module(); break;
      case M2_DIODE_RAW:         m2_diode_module_raw(); break;
      case M2_TRAN_MODULE_VDS:   m2_tran_module_vds(); break;
      case M2_TRAN_MODULE_VGS:   m2_tran_module_vgs(); break;
      case M2_TRAN_RAW:          m2_tran_module_raw(); break;
      case M3_LED_FLASH:         m3_led_flash(); break;
      case M3_PPG:               m3_ppg(); break;
      case M3_COLLECT_CURRENT:   m3_collect_current(); break;
      case M3_STEP_LED:          m3_step_led_brightness(); break;
      case LIFI_TX:              lifi_tx(); break;
      case LIFI_RX:              lifi_rx(); break;
      case EXERCISE_CLASSIFY:    detect_exercise_type(); break;
      case HEARTBEAT_LED:        heartbeat_led(); break;
      case EXIT_COMMAND:         break;
      default:
        Serial.println(F("Unknown command"));
        break;
    }
  }
}

// ----------------------------------------------------

Command mapStringToCommand(String command) {
  if (command == "m1na") return M1_NETWORK_ARRAY;
  if (command == "m1vd") return M1_VOLTAGE_DIVIDER;
  if (command == "m2d") return M2_DIODE_MODULE;
  if (command == "m2draw") return M2_DIODE_RAW;
  if (command == "m2tran_vgs") return M2_TRAN_MODULE_VGS;
  if (command == "m2tran_vds") return M2_TRAN_MODULE_VDS;
  if (command == "m2tranraw") return M2_TRAN_RAW;
  if (command == "m3led") return M3_LED_FLASH;
  if (command == "m3ppg") return M3_PPG;
  if (command == "m3pdcurrent") return M3_COLLECT_CURRENT;
  if (command == "m3stepled") return M3_STEP_LED;
  if (command == "lifi_tx") return LIFI_TX;
  if (command == "lifi_rx") return LIFI_RX;
  if (command == "exercise") return EXERCISE_CLASSIFY;
  if (command == "heartbeat") return HEARTBEAT_LED;
  if (command == "exit" || command == "e") return EXIT_COMMAND;
  return UNKNOWN_COMMAND;
}

// ----------------------------------------------------

bool exit_condition() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "exit" || command == "e") {
      reset_dac();
      Serial.println(F("Ready to enter next command"));
      return true;
    }
  }
  return false;
}

// ----------------------------------------------------
// HEARTBEAT LED MODE
// ----------------------------------------------------

void heartbeat_led() {
  Serial.println(F("Heartbeat LED mode started."));
  Serial.println(F("Place your finger on Module 3 sensor..."));

  ppg.begin();
  ppg.setup();
  ppg.setPulseAmplitudeRed(0x7F);
  ppg.setPulseAmplitudeIR(0x7F);
  ppg.setPulseAmplitudeGreen(0x00);
  ppg.setPulseAmplitudeBlue(0x00);

  pinMode(HEART_LED_PIN, OUTPUT);

  float dc = 0;
  const float alpha = 0.95;
  float prev2 = 0, prev1 = 0, cur = 0;
  const float THRESH = 5.0;

  unsigned long lastBeat = 0;
  const unsigned long MIN_BEAT_INTERVAL = 300;
  unsigned long lastSample = 0;

  while (true) {
    if (exit_condition()) return;

    if (millis() - lastSample < 10) continue;
    lastSample = millis();

    prev2 = prev1;
    prev1 = cur;

    int32_t ir = ppg.getIR();

    dc = dc + alpha * (ir - dc);
    cur = ir - dc;

    unsigned long now = millis();

    bool isPeak =
      (prev1 > prev2) &&
      (prev1 > cur) &&
      (prev1 > THRESH) &&
      ((now - lastBeat) > MIN_BEAT_INTERVAL);

    if (isPeak) {
      lastBeat = now;

      digitalWrite(HEART_LED_PIN, HIGH);
      delay(40);
      digitalWrite(HEART_LED_PIN, LOW);

      Serial.println(F("BEAT"));
    }
  }
}

// ----------------------------------------------------
// YOUR ORIGINAL LAB FUNCTIONS
// ----------------------------------------------------

void dac_setup() {
  initADAC(0x10, 1, 1);
  setDACGain(1);
  setDACpin(dac_diode);
  setDACpin(dac_nmos_gate);
  setDACpin(dac_nmos_drain);
  setDACpin(dac_pd_pair1);
  setDACpin(dac_pd_pair2);
}

void reset_dac() {
  setDACVal(dac_diode, 0);
  setDACVal(dac_nmos_gate, 0);
  setDACVal(dac_nmos_drain, 0);
  setDACVal(dac_pd_pair1, 0);
  setDACVal(dac_pd_pair2, 0);
}

void m1_network_array() {
  Serial.println(F("Make sure that module 1 is active"));
  while (1) {
    int analog_value = analogRead(m1_na);
    if (analog_value > 1000 && analog_value < 1020) Serial.println(F("S1 (3.3V)"));
    else if (analog_value > 590 && analog_value < 615) Serial.println(F("S2 (1.95V)"));
    else if (analog_value > 360 && analog_value < 375) Serial.println(F("S3 (1.18V)"));
    else if (analog_value > 755 && analog_value < 775) Serial.println(F("S4 (2.50V)"));
    else if (analog_value > 500 && analog_value < 520) Serial.println(F("S5 (1.65V)"));
    else if (analog_value > 240 && analog_value < 260) Serial.println(F("S6 (0.804V)"));
    else if (analog_value > 635 && analog_value < 660) Serial.println(F("S7 (2.12V)"));
    else if (analog_value > 400 && analog_value < 425) Serial.println(F("S8 (1.35V)"));
    else if (analog_value > 0 && analog_value < 200) Serial.println(F("S9 (0V)"));

    if (exit_condition()) break;
    delay(20);
  }
}

void m1_voltagedivider() {
  Serial.println(F("Make sure that module 1 is active"));
  while (1) {
    Serial.print((analogRead(m1_vd) / 1023.0) * ref_voltage);
    Serial.println(F("V"));
    if (exit_condition()) break;
  }
}

void m2_diode_module() {
  Serial.println(F("Make sure that module 2 is active"));
  int inc_interval = 10;
  int inc_delay = 1;
  int inc = 0;
  unsigned long prev_inc_time = 0;

  Serial.println(F("Diode current in (mA), Voltage applied across diode in (V)"));
  while (1) {
    if ((inc <= dac_3p3v) && (millis() - prev_inc_time > (unsigned long)inc_delay)) {
      prev_inc_time = millis();
      setDACVal(dac_diode, inc);
      float resistor_voltage = (analogRead(m2_diode) * adc_conversion);
      Serial.print(resistor_voltage);
      Serial.print(",");
      float diode_voltage = inc * 0.00165;
      Serial.print(diode_voltage);
      Serial.print(",");
      Serial.println(diode_voltage - resistor_voltage);
      inc = inc + inc_interval;
    }
    if ((inc >= dac_3p3v) && (millis() - prev_inc_time > (unsigned long)inc_delay)) {
      prev_inc_time = millis();
      setDACVal(dac_diode, 0);
      inc = 0;
      Serial.println(F("Ready to enter next command"));
      break;
    }
  }
}

void m2_diode_module_raw() {
  Serial.println(F("Make sure that module 2 is active"));
  int inc_interval = 10;
  int inc_delay = 1;
  int inc = 0;
  unsigned long prev_inc_time = 0;

  Serial.println(F("Current in mA"));
  while (1) {
    if ((inc <= dac_3p3v) && (millis() - prev_inc_time > (unsigned long)inc_delay)) {
      prev_inc_time = millis();
      setDACVal(dac_diode, inc);
      Serial.println(analogRead(m2_diode) * adc_conversion);
      inc = inc + inc_interval;
    }
    if ((inc >= dac_3p3v) && (millis() - prev_inc_time > (unsigned long)inc_delay)) {
      prev_inc_time = millis();
      setDACVal(dac_diode, 0);
      inc = 0;
      Serial.println(F("Ready to enter next command"));
      break;
    }
  }
}

void m2_tran_module_vds() {
  Serial.println(F("Make sure that module 2 is active"));
  Serial.println(F("Vgs | Vds | Ic"));
  while (1) {
    for (int k = 0; k <= 2000; k = k + 200) {
      setDACVal(dac_nmos_gate, k);
      for (int i = 0; i <= 2000; i = i + 10) {
        setDACVal(dac_nmos_drain, i);
        Serial.print(k * dac_conversion);
        Serial.print(F(", "));
        Serial.print(i * dac_conversion);
        Serial.print(F(", "));
        Serial.println(analogRead(m2_nmos) * adc_conversion);
        delay(5);
      }
    }
    Serial.println(F("Ready to enter next command"));
    break;
  }
}

void m2_tran_module_vgs() {
  Serial.println(F("Make sure that module 2 is active"));
  Serial.println(F("Vds | Vgs | Ic"));
  while (1) {
    for (int k = 0; k <= 2000; k = k + 200) {
      setDACVal(dac_nmos_drain, k);
      for (int i = 0; i <= 2000; i = i + 10) {
        setDACVal(dac_nmos_gate, i);
        Serial.print(k * dac_conversion);
        Serial.print(F(", "));
        Serial.print(i * dac_conversion);
        Serial.print(F(", "));
        Serial.println(analogRead(m2_nmos) * adc_conversion);
        delay(5);
      }
    }
    Serial.println(F("Ready to enter next command"));
    break;
  }
}

void m2_tran_module_raw() {
  Serial.println(F("Make sure that module 2 is active"));
  while (1) {
    for (int k = 0; k < 2000; k = k + 200) {
      setDACVal(dac_nmos_gate, k);
      Serial.print(F("Gate voltage Vgs = "));
      Serial.print(k * dac_conversion);
      Serial.println(F(" V"));
      for (int i = 0; i < 2000; i = i + 10) {
        setDACVal(dac_nmos_drain, i);
        Serial.println(analogRead(m2_nmos) * adc_conversion);
        delay(5);
      }
    }
    Serial.println(F("Ready to enter next command"));
    break;
  }
}

void m3_led_flash() {
  Serial.println(F("Make sure that module 3 is active"));
  int time_period = 100;
  int previous_state = 0;
  unsigned long previous_toggle_time = 0;

  while (1) {
    Serial.println(analogRead(m3_pd));
    if (millis() - previous_toggle_time >= (unsigned long)(time_period / 2)) {
      previous_toggle_time = millis();
      if (previous_state == 0) {
        setDACVal(dac_pd_pair1, 2250);
        previous_state = 1;
      } else {
        setDACVal(dac_pd_pair1, 0);
        previous_state = 0;
      }
    }
    if (exit_condition()) break;
  }
}

void m3_collect_current() {
  int count = 0;
  int incrementer = 10;
  int analog_value = 0;
  setDACVal(dac_pd_pair1, 2250);
  while (count < 3000) {
    analog_value = analogRead(m3_pd) * adc_conversion * 1000;
    Serial.println(analog_value / 3);
    count = count + incrementer;
    delay(incrementer);
  }
  setDACVal(dac_pd_pair1, 0);
  Serial.println(F("Ready to enter next command"));
}

void m3_step_led_brightness() {
  int led_brightness = 1650;
  int max_led_brightness = 2000;
  int brightness_incrementer = 50;

  int increment_counter = 0;
  int pd_current = 0;
  while (1) {
    Serial.print(led_brightness * dac_conversion);
    Serial.print(",");
    pd_current = analogRead(m3_pd) * adc_conversion * 1000;
    Serial.println(pd_current / 3);
    if (led_brightness <= max_led_brightness) {
      increment_counter = increment_counter + 1;
      if (increment_counter == brightness_incrementer) {
        setDACVal(dac_pd_pair1, led_brightness);
        led_brightness = led_brightness + brightness_incrementer;
        increment_counter = 0;
      }
    } else break;
    if (exit_condition()) break;
    delay(10);
  }
  setDACVal(dac_pd_pair1, 0);
  Serial.println(F("Ready to enter next command"));
}

void m3_ppg() {
  ppg.begin();
  ppg.setup();
  ppg.setPulseAmplitudeRed(0x7F);
  ppg.setPulseAmplitudeIR(0x7F);
  ppg.setPulseAmplitudeGreen(0x7F);
  ppg.setPulseAmplitudeBlue(0x7F);

  while (1) {
    Serial.print(ppg.getIR());
    Serial.print(",");
    Serial.println(ppg.getRed());
    if (exit_condition()) break;
  }
}

bool read_pd() {
  int pd_value = analogRead(m3_pd);
  return pd_value > THRESHOLD;
}

void lifi_tx() {
  char* string = (char*)"This is a test transmission! ";
  int string_length = strlen(string);
  while (1) {
    for (int i = 0; i < string_length; i++) {
      send_byte(string[i]);
    }
    delay(1000);
    if (exit_condition()) break;
  }
}

void send_byte(char my_byte) {
  setDACVal(dac_pd_pair2, 0);
  delay(PERIOD);

  for (int i = 0; i < 8; i++) {
    if ((my_byte & (0x01 << i)) != 0) setDACVal(dac_pd_pair2, 2250);
    else setDACVal(dac_pd_pair2, 0);
    delay(PERIOD);
  }

  setDACVal(dac_pd_pair2, 2250);
  delay(PERIOD);
}

void lifi_rx() {
  Serial.println(F("Make sure that module 3 is active"));
  bool previous_state = false;
  bool current_state = false;
  while (1) {
    current_state = read_pd();
    if (!current_state && previous_state) {
      print_byte(get_byte());
    }
    previous_state = current_state;
    if (exit_condition()) break;
  }
}

char get_byte() {
  char ret = 0;
  delay(PERIOD * 1.5);
  for (int i = 0; i < 8; i++) {
    ret = ret | (read_pd() << i);
    delay(PERIOD);
  }
  return ret;
}

void print_byte(char my_byte) {
  char buff[2];
  sprintf(buff, "%c", my_byte);
  Serial.print(buff);
}

/* ------------------------------------------------------------------
  Identify exercise type (aerobic vs anaerobic)
 ------------------------------------------------------------------ */
void detect_exercise_type() {
  Serial.println(F("Make sure Module 3 is active and the PPG sensor is on your finger."));
  Serial.println(F("Try to keep your finger steady. After exercise, put your finger on the sensor for ~30 seconds."));
  Serial.println(F("Collecting PPG data..."));

  ppg.begin();
  ppg.setup();
  ppg.setPulseAmplitudeRed(0x7F);
  ppg.setPulseAmplitudeIR(0x7F);
  ppg.setPulseAmplitudeGreen(0x7F);
  ppg.setPulseAmplitudeBlue(0x7F);

  const unsigned long MEASUREMENT_TIME_MS   = 30000;
  const unsigned long MIN_BEAT_INTERVAL_MS  = 350;
  const unsigned long SAMPLE_PERIOD_MS      = 10;
  const int           MAX_BEATS             = 40;   // reduced to save RAM

  unsigned long beat_timestamps[MAX_BEATS];
  int beat_count = 0;

  float dc_estimate = 0.0f;
  const float alpha = 0.95f;

  float ac_prev2 = 0.0f;
  float ac_prev1 = 0.0f;
  float ac_cur   = 0.0f;

  unsigned long last_beat_time   = 0;
  unsigned long start_time       = millis();
  unsigned long last_sample_time = millis();

  for (int i = 0; i < 3; i++) {
    int32_t ir = ppg.getIR();
    dc_estimate = dc_estimate + alpha * (ir - dc_estimate);
    float ac = ir - dc_estimate;

    if (i == 0)      ac_prev2 = ac;
    else if (i == 1) ac_prev1 = ac;
    else             ac_cur   = ac;

    delay(SAMPLE_PERIOD_MS);
  }

  const float AC_THRESHOLD = 5.0f;

  while (millis() - start_time < MEASUREMENT_TIME_MS) {
    if (exit_condition()) {
      Serial.println(F("Measurement aborted by user."));
      return;
    }

    if (millis() - last_sample_time < SAMPLE_PERIOD_MS) {
      continue;
    }
    last_sample_time = millis();

    ac_prev2 = ac_prev1;
    ac_prev1 = ac_cur;

    int32_t ir = ppg.getIR();
    dc_estimate = dc_estimate + alpha * (ir - dc_estimate);
    ac_cur = ir - dc_estimate;

    unsigned long now = millis();

    bool is_peak = (ac_prev1 > ac_prev2) &&
                   (ac_prev1 >= ac_cur) &&
                   (ac_prev1 > AC_THRESHOLD);

    if (is_peak && (now - last_beat_time) > MIN_BEAT_INTERVAL_MS) {
      if (beat_count < MAX_BEATS) {
        beat_timestamps[beat_count] = now;
        beat_count++;
      }
      last_beat_time = now;
    }
  }

  if (beat_count < 3) {
    Serial.print(F("Not enough beats detected. Beats counted: "));
    Serial.println(beat_count);
    Serial.println(F("Try keeping still or restarting, then run 'exercise' again."));
    return;
  }

  float sum_bpm    = 0.0f;
  float sum_bpm_sq = 0.0f;
  int   interval_count = beat_count - 1;

  for (int i = 1; i < beat_count; i++) {
    unsigned long dt = beat_timestamps[i] - beat_timestamps[i - 1];
    if (dt == 0) continue;
    float bpm = 60000.0f / (float)dt;
    sum_bpm    += bpm;
    sum_bpm_sq += bpm * bpm;
  }

  float mean_bpm = sum_bpm / interval_count;
  float var_bpm  = (sum_bpm_sq / interval_count) - (mean_bpm * mean_bpm);
  if (var_bpm < 0) var_bpm = 0;
  float std_bpm  = sqrt(var_bpm);

  Serial.print(F("Average heart rate over ~30 s: "));
  Serial.print(mean_bpm, 1);
  Serial.println(F(" BPM"));

  Serial.print(F("Approx HRV (std dev of BPM): "));
  Serial.print(std_bpm, 1);
  Serial.println(F(" BPM"));

  String exercise_type;
  if (mean_bpm >= 140.0f) {
    exercise_type = "Anaerobic (high-intensity, short bursts)";
  } else if (mean_bpm >= 90.0f) {
    exercise_type = "Aerobic (moderate-intensity, sustainable)";
  } else {
    exercise_type = "Rest / very light activity (no clear exercise)";
  }

  Serial.print(F("Estimated exercise type: "));
  Serial.println(exercise_type);

  Serial.println(F("Examples:"));
  if (mean_bpm >= 140.0f) {
    Serial.println(F("Slow down heartrate → walking, light jogging, dancing"));
    Serial.println(F("Maintain heartrate → sprinting, biking"));
  } else if (mean_bpm >= 90.0f && mean_bpm < 140.0f) {
    Serial.println(F("Maintain heartrate → walking, light jogging"));
    Serial.println(F("Increase heartrate → sprinting, running, biking, dancing"));
  } else {
    Serial.println(F("Increase heartrate!"));
  }

  Serial.println(F("Ready to enter next command"));
}
