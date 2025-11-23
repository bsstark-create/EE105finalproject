void detect_exercise_type() {

  Serial.println("Place finger on the PPG sensor. Stay still.");
  Serial.println("Collecting 30 seconds of PPG...");

  // Initialize PPG
  ppg.begin();
  ppg.setup();
  ppg.setPulseAmplitudeRed(0x7F);
  ppg.setPulseAmplitudeIR(0x7F);

  // Heartbeat LED output
  const int HEART_LED_PIN = dac_pd_pair2;
  setDACVal(HEART_LED_PIN, 0);

  const unsigned long MEAS_MS = 30000;
  const unsigned long SAMPLE_MS = 10;
  const unsigned long MIN_BEAT_MS = 300;  // ~200 BPM max

  unsigned long beat_time[100];
  int beat_idx = 0;

  float dc = 0;
  const float alpha = 0.98;

  float a2 = 0, a1 = 0, ac = 0;

  unsigned long start = millis();
  unsigned long last_sample = millis();
  unsigned long last_beat = 0;

  const float PEAK_THRESH = 6.0;

  // Initialize the AC buffer
  for (int i = 0; i < 3; i++) {
    int ir = ppg.getIR();
    dc = dc + alpha * (ir - dc);
    ac = ir - dc;
    if (i == 0) a2 = ac;
    if (i == 1) a1 = ac;
    if (i == 2) ac = ac;
    delay(SAMPLE_MS);
  }

  // --- Sampling Loop ---
  while (millis() - start < MEAS_MS) {

    if (millis() - last_sample < SAMPLE_MS) continue;
    last_sample = millis();

    // shift window
    a2 = a1;
    a1 = ac;

    int ir = ppg.getIR();
    dc = dc + alpha * (ir - dc);
    ac = ir - dc;

    unsigned long now = millis();

    bool peak = (a1 > a2) && (a1 >= ac) && (a1 > PEAK_THRESH);

    if (peak && (now - last_beat) > MIN_BEAT_MS) {

      // Save beat
      if (beat_idx < 100) {
        beat_time[beat_idx++] = now;
      }

      last_beat = now;

      // ❤️ LED blink on heartbeat
      setDACVal(HEART_LED_PIN, 2000);
      delay(25);
      setDACVal(HEART_LED_PIN, 0);
    }
  }

  // Too few beats?
  if (beat_idx < 3) {
    Serial.println("Not enough beats detected. Try again.");
    return;
  }

  // Compute BPM and HRV
  float sumBPM = 0;
  float sumBPM2 = 0;
  int N = beat_idx - 1;

  for (int i = 1; i < beat_idx; i++) {
    float dt = beat_time[i] - beat_time[i - 1];
    float bpm = 60000.0 / dt;

    sumBPM += bpm;
    sumBPM2 += bpm * bpm;
  }

  float meanBPM = sumBPM / N;
  float varBPM = (sumBPM2 / N) - (meanBPM * meanBPM);
  if (varBPM < 0) varBPM = 0;
  float stdBPM = sqrt(varBPM);

  Serial.print("Heart Rate: ");
  Serial.print(meanBPM, 1);
  Serial.println(" BPM");

  Serial.print("HRV (std dev): ");
  Serial.print(stdBPM, 1);
  Serial.println(" BPM");

  // ================================================================
  //              EXERCISE CLASSIFICATION (PROJECT SPEC)
  // ================================================================
  // The project description says:
  // Aerobic  → moderate intensity, stable HR, longer duration
  // Anaerobic→ high intensity, high HR, short bursts, unstable HR

  String type;

  if (meanBPM > 140 && stdBPM > 10) {
    type = "Anaerobic (high intensity, short bursts)";
  }
  else if (meanBPM > 90 && stdBPM <= 10) {
    type = "Aerobic (moderate, sustained)";
  }
  else {
    type = "Rest / light activity";
  }

  Serial.print("Estimated Exercise Type: ");
  Serial.println(type);

  Serial.println("Examples:");
  if (type.startsWith("Anaerobic")) {
    Serial.println("• Sprinting, HIIT, Weightlifting, Jumping");
  } else if (type.startsWith("Aerobic")) {
    Serial.println("• Jogging, Swimming, Cycling, Dancing, Walking");
  } else {
    Serial.println("No clear exercise detected.");
  }

  Serial.println("Ready for next command.");
}
