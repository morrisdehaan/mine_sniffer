const int DETECTORS = 3;
// TODO: explain
const int CALIBRATION_STEPS = 64;

// used to send pulses to charge the capacitors
const byte PULSE_PINS[DETECTORS] = { A0, A2, A4 };
// used to measure capacitor charge
const byte CAP_PINS[DETECTORS] = { A1, A3, A5 };
// used for visually testing detectors
const byte LED_PINS[DETECTORS] = { 12, 11, 10 };

const int MEASUREMENTS = 256;
const byte PULSES = 12;

// running sums of sums of CALIBRATION_STEPS sums
long int sum_sums[DETECTORS] = { 0, 0, 0 };
// each separate sum of sum_sums
long int sum_sums_each[DETECTORS][CALIBRATION_STEPS] = { 0 };
// current index within sum_sums_each
int sum_sums_idx = 0;

void setup() {
  for (byte p : PULSE_PINS) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  for (byte c : CAP_PINS) {
    pinMode(c, INPUT);
  }
  for (byte l : LED_PINS) {
    pinMode(l, OUTPUT);
    digitalWrite(l, LOW);
  }

  Serial.begin(9600);
}

void loop() {
  long unsigned int sums[DETECTORS] = { 0, 0, 0 };
  int mins[DETECTORS] = { 2000, 2000, 2000 };
  int maxs[DETECTORS] = { 0, 0, 0 };

  // measure max and min peaks, as well as computing the total capacitor charge sum
  for (int im = 0; im < MEASUREMENTS; im++) {
    // reset capacitors
    for (byte c : CAP_PINS) {
      pinMode(c, OUTPUT);
      digitalWrite(c, LOW);
    }
    delayMicroseconds(20);

    for (byte c : CAP_PINS) {
      pinMode(c, INPUT);
    }

    // apply pulses
    for (int ip = 0; ip < PULSES; ip++) {
      for (byte p : PULSE_PINS) {
        digitalWrite(p, HIGH);
      }
      delayMicroseconds(3);
      for (byte p : PULSE_PINS) {
        digitalWrite(p, LOW);
      }
      delayMicroseconds(3);
    }

    // read capacitor values
    for (int i = 0; i < DETECTORS; i++) {
      int val = analogRead(CAP_PINS[i]);

      sums[i] += val;
      mins[i] = min(val, mins[i]);
      maxs[i] = max(val, maxs[i]);
    }
  }

  long int diffs[DETECTORS] = { 0 };

  for (int i = 0; i < DETECTORS; i++) {
    // remove spikes
    sums[i] -= mins[i] + maxs[i];

    // * compute running sum *
    // first sum of the running sum
    long int first_sum = sum_sums_each[i][sum_sums_idx];
    // update running sum
    sum_sums[i] += sums[i] - first_sum;
    sum_sums_each[i][sum_sums_idx] = sums[i];

    sum_sums_idx = (sum_sums_idx + 1) % CALIBRATION_STEPS;

    // * turn on/off LED *
    long int avg_sum = sum_sums[i] / CALIBRATION_STEPS;
    diffs[i] = sums[i] - avg_sum;
    
    if (diffs[i] < -50) {
      digitalWrite(LED_PINS[i], HIGH);
      delayMicroseconds(10000);
      digitalWrite(LED_PINS[i], LOW);
    } else {
      digitalWrite(LED_PINS[i], LOW);
    }
  }

  // TODO: wtf 
  // send all data at once,
  //  I couldn't be bothered to write anything fancy with Serial.write (would require raspberrypi/arduino syncing with headers or smth)
  //  so good ol' Serial.println will suffice
  for (int i = 0; i < DETECTORS; i++) {
    Serial.print(diffs[i]);
    Serial.print(" ");
  }
  Serial.print("\n");
}
