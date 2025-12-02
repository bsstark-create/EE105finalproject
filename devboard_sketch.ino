#include <Arduino.h>
#include "AD5593R.h"
#include <Wire.h>
#include "MAX86916_eda.h"

#define m3_pd A6      
#define module_selector A7  

#define dac_diode 0         
#define dac_nmos_gate 1     
#define dac_nmos_drain 2    
#define dac_pd_pair1 3      
#define dac_pd_pair2 4      

#define dac_0v 0        
#define dac_1p8v 1150   
#define dac_3p3v 2000   

#define adc_conversion 0.00322
#define dac_conversion 0.00165

#define PERIOD 3
#define THRESHOLD 200

#define ref_voltage 3.3

MAX86916_eda ppg;

// stores sample values
const int MAX_SAMPLES = 3000;
float stored_ac[MAX_SAMPLES];
unsigned long stored_time[MAX_SAMPLES];
int stored_sample_count = 0;

unsigned long stored_beats[200];
int stored_beat_count = 0;

float stored_mean_bpm = 0;

String stored_exercise_type = "";
bool sample_available = false;

enum Command {
  M3_PPG,
  EXERCISE_CLASSIFY,   // exercise type classification from PPG
  SAMPLE_DATA,         // lets you see the sample of the collected data
  EXIT_COMMAND
};

void dac_setup();
void reset_dac();
Command mapStringToCommand(String command);
bool exit_condition();
void m3_ppg();
void detect_exercise_type();
void print_sample_data();

void setup() {
  Wire.begin();
  Serial.begin(115200);
  dac_setup();
  reset_dac();
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n'); // Read the incoming command
    Command cmd = mapStringToCommand(command);

    switch (cmd) {
      case M3_PPG:
        m3_ppg();
        break;
      case EXERCISE_CLASSIFY:
        detect_exercise_type();
        break;
      case SAMPLE_DATA:
        print_sample_data();
        break;
      case EXIT_COMMAND:
        break;
      default:
        Serial.println("Unknown command");
        break;
    }
  }
}

Command mapStringToCommand(String command) {
  if (command == "m3ppg")   return M3_PPG;
  if (command == "exercise") return EXERCISE_CLASSIFY;
  if (command == "sample")   return SAMPLE_DATA;
  if (command == "exit" || command == "e") return EXIT_COMMAND;
  return EXIT_COMMAND;  // default, will show "Unknown command" in loop
}

bool exit_condition() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    if (command == "exit" || command == "e") {
      reset_dac();
      Serial.println("Ready to enter next command");
      return true;
    }
  }
  return false;
}

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
  setDACVal(dac_diode,0);
  setDACVal(dac_nmos_gate,0);
  setDACVal(dac_nmos_drain,0);
  setDACVal(dac_pd_pair1,0);
  setDACVal(dac_pd_pair2,0);
}

void m3_ppg() {
  int16_t blue;
  int16_t green;
  int16_t IR;
  int16_t red;
  int16_t IR_blue;
  int16_t IR_red;
  int16_t IR_green;

  ppg.begin();
  ppg.setup();
  ppg.setPulseAmplitudeRed(0xFF);
  ppg.setPulseAmplitudeIR(0xFF);
  ppg.setPulseAmplitudeGreen(0xFF);
  ppg.setPulseAmplitudeBlue(0xFF);

  while (1) {
    Serial.print(ppg.getIR());  // IR
    Serial.print(",");
    Serial.println(ppg.getRed()); // Red
    if (exit_condition()) break;
  }
}

void detect_exercise_type() {
  Serial.println("Make sure Module 3 is active and the PPG sensor is on your finger.");
  Serial.println("Try to keep your finger steady. After completing a set of your exercise (i.e., running, weightlifting), put your finger on the sensor for ~30 seconds.");
  Serial.println("Collecting PPG data for ~30 seconds...");

  // Initialize PPG like in m3_ppg()
  ppg.begin();
  ppg.setup();
  ppg.setPulseAmplitudeRed(0xFF);
  ppg.setPulseAmplitudeIR(0xFF);
  ppg.setPulseAmplitudeGreen(0xFF);
  ppg.setPulseAmplitudeBlue(0xFF);

  const unsigned long MEASUREMENT_TIME_MS   = 30000UL;  //~30 seconds
  const unsigned long MIN_BEAT_INTERVAL_MS  = 250UL;    //~240 BPM max guard
  const unsigned long SAMPLE_PERIOD_MS      = 10UL;     //~80 Hz sampling
  const int           MAX_BEATS             = 200;

  unsigned long beat_timestamps[MAX_BEATS];
  int beat_count = 0;

  // removes DC
  float dc_estimate = 0.0;
  const float alpha = 0.97;

  // peak detection
  float ac_prev2 = 0.0;
  float ac_prev1 = 0.0;
  float ac_cur   = 0.0;

  unsigned long last_beat_time   = 0;
  unsigned long start_time       = millis();
  unsigned long last_sample_time = millis();

  // Reset storage 
  stored_sample_count = 0;
  stored_beat_count   = 0;
  sample_available    = false;

  // Prime with 3 samples
  for (int i = 0; i < 3; i++) {
    int32_t ir = ppg.getIR();
    dc_estimate = dc_estimate + alpha * (ir - dc_estimate);
    float ac = (float)ir - dc_estimate;

    if (i == 0)      ac_prev2 = ac;
    else if (i == 1) ac_prev1 = ac;
    else             ac_cur   = ac;

    delay(SAMPLE_PERIOD_MS);
  }

  const float AC_THRESHOLD = 10.0;

  // main sample loop
  while (millis() - start_time < MEASUREMENT_TIME_MS) {
    if (exit_condition()) {
      Serial.println("Measurement aborted by user.");
      return;
    }

    if (millis() - last_sample_time < SAMPLE_PERIOD_MS) {
      continue;
    }
    last_sample_time = millis();

    // shift window
    ac_prev2 = ac_prev1;
    ac_prev1 = ac_cur;

    // new sample
    int32_t ir = ppg.getIR();
    dc_estimate = dc_estimate + alpha * (ir - dc_estimate);
    ac_cur = (float)ir - dc_estimate;

    unsigned long now = millis();

    // store each sample in globals
    if (stored_sample_count < MAX_SAMPLES) {
      stored_ac[stored_sample_count]   = ac_cur;           // AC value
      stored_time[stored_sample_count] = now - start_time; // ms from start
      stored_sample_count++;
    }

    // detect local peak 
    bool is_peak = (ac_prev1 > ac_prev2) && (ac_prev1 >= ac_cur) && (ac_prev1 > AC_THRESHOLD);

    if (is_peak && (now - last_beat_time) > MIN_BEAT_INTERVAL_MS) {
      if (beat_count < MAX_BEATS) {
        beat_timestamps[beat_count] = now;
        beat_count++;
      }

      if (stored_beat_count < 200) {
        stored_beats[stored_beat_count] = now - start_time; // ms from start
        stored_beat_count++;
      }

      last_beat_time = now;
    }
  } // end sampling

  if (stored_beat_count < 3) {
    Serial.print("Not enough beats detected. Beats counted: ");
    Serial.println(stored_beat_count);
    Serial.println("Try keeping still, ensure good contact, or increase measurement time and run 'exercise' again.");
    return;
  }

  // mean bpm
  float mean_bpm = stored_beat_count * 2.0;

  // threshold values
  String exercise_type;
  if (mean_bpm >= 130.0) {
    exercise_type = "Anaerobic (high-intensity, short bursts)";
  } else if (mean_bpm >= 70.0) {
    exercise_type = "Aerobic (moderate-intensity, sustainable)";
  } else {
    exercise_type = "Rest / very light activity (no clear exercise)";
  }

  Serial.print("Estimated exercise type (based on heart rate intensity): ");
  Serial.println(exercise_type);

  // store stats in globals for the "sample" command
  stored_mean_bpm       = mean_bpm;
  stored_exercise_type  = exercise_type;
  sample_available      = true;

  Serial.println();
  Serial.println("Ready to enter next command");
}

void print_sample_data() {
  if (!sample_available) {
    Serial.println("No PPG sample collected yet. Run 'exercise' first.");
    return;
  }

  Serial.println("---Stored PPG sample data---");

  // Print all stored samples
  for (int i = 0; i < stored_sample_count; i++) {
    Serial.print(stored_time[i]);
    Serial.print(",");
    Serial.println(stored_ac[i]);
  }
  Serial.println("Time(ms), AC_value");

  Serial.print("Total samples: ");
  Serial.println(stored_sample_count);

  Serial.print("Total detected beats: ");
  Serial.println(stored_beat_count);

  Serial.print("Mean BPM: ");
  Serial.println(stored_beat_count * 2);

  Serial.print("Exercise Type: ");
  Serial.println(stored_exercise_type);
  Serial.println();
}
