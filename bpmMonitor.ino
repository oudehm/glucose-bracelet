/**
 * hrv_max30102.ino
 * 
 * Reads raw PPG (photoplethysmography) data from a MAX30102 pulse oximeter
 * sensor and computes real-time heart rate (BPM) and HRV (RMSSD).
 * 
 * Hardware:
 *   - Seeed XIAO ESP32-S3
 *   - HiLetgo MAX30102 breakout module
 * 
 * Wiring:
 *   MAX30102 VIN  → 3.3V
 *   MAX30102 GND  → GND
 *   MAX30102 SDA  → GPIO5
 *   MAX30102 SCL  → GPIO6
 * 
 * Library: SparkFun MAX3010x (install via Arduino Library Manager)
 * 
 * Usage:
 *   Flash, open Serial Monitor at 115200 baud, rest fingertip flat on the
 *   sensor window (black side), and hold still for ~5 seconds to calibrate.
 *   RR intervals and RMSSD will print once beats are detected.
 */

#include <Wire.h>
#include "MAX30105.h"

MAX30105 particleSensor;

// ── Beat detection parameters ─────────────────────────────────────────────────

// Minimum and maximum RR interval in ms.
// MIN_RR_MS = 500ms → rejects anything faster than 120 BPM (avoids double triggers).
// MAX_RR_MS = 1500ms → rejects anything slower than 40 BPM (avoids missed beats).
const long MIN_RR_MS = 500;
const long MAX_RR_MS = 1500;

// After detecting a peak, ignore the signal for this many ms.
// Prevents the same heartbeat from being counted twice on the way down.
const long REFRACTORY_MS = 300;

// ── RR interval buffer ────────────────────────────────────────────────────────

// Stores the last N RR intervals (ms) in a circular buffer.
// Used to compute RMSSD over a rolling window.
const int RR_BUFFER_SIZE = 20;
long rrBuffer[RR_BUFFER_SIZE];
int rrCount = 0;  // total beats detected so far (not capped at buffer size)

// ── State variables ───────────────────────────────────────────────────────────

long lastPeakTime = 0;  // millis() timestamp of the last detected peak
bool rising = false;    // true while IR signal is above the dynamic threshold

// Dynamic min/max tracking — updated every sample via exponential smoothing.
// This lets the threshold follow slow baseline drift caused by finger movement
// or LED warming up, without reacting to fast heartbeat swings.
long dynamicMin = 999999;
long dynamicMax = 0;


void setup() {
  Serial.begin(115200);

  // Initialize I2C on XIAO ESP32-S3 pins (default Wire pins are 5/6)
  Wire.begin(5, 6);

  // Try to connect to the MAX30102 (I2C address 0x57)
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found. Check wiring.");
    while (1);  // halt — no point continuing without the sensor
  }

  // Use the library's default configuration, then set LED brightness manually.
  // 0x1F is a moderate brightness that works well for fingertip placement.
  // If the signal is too weak, increase toward 0xFF.
  // If it's saturating (flat waveform), decrease toward 0x0F.
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);

  Serial.println("Ready. Rest finger on sensor and hold still for 5 seconds.");
}


void loop() {
  // check() pulls new samples from the sensor's FIFO into the library's buffer.
  // Must be called regularly — without it, available() always returns 0.
  particleSensor.check();

  while (particleSensor.available()) {
    long ir  = particleSensor.getIR();  // raw IR channel value (18-bit ADC)
    long now = millis();

    // ── Dynamic threshold ─────────────────────────────────────────────────────
    // Track the running min and max using exponential smoothing (α = 0.01).
    // The threshold sits at the midpoint, rising and falling with the baseline.
    dynamicMin = 0.99 * dynamicMin + 0.01 * ir;
    dynamicMax = 0.99 * dynamicMax + 0.01 * ir;
    if (ir < dynamicMin) dynamicMin = ir;
    if (ir > dynamicMax) dynamicMax = ir;

    long threshold = (dynamicMin + dynamicMax) / 2;

    // ── Peak detection ────────────────────────────────────────────────────────
    // A peak is the first sample above the threshold after the signal was below it.
    // The refractory period prevents re-triggering during the falling edge.
    if (ir > threshold) {
      if (!rising && (now - lastPeakTime) > REFRACTORY_MS) {
        rising = true;
        long rr = now - lastPeakTime;  // time since last peak = RR interval

        // Only accept RR intervals in the physiologically plausible range
        if (lastPeakTime > 0 && rr > MIN_RR_MS && rr < MAX_RR_MS) {

          // Store in circular buffer (overwrites oldest entry when full)
          rrBuffer[rrCount % RR_BUFFER_SIZE] = rr;
          rrCount++;

          Serial.print("RR="); Serial.print(rr);
          Serial.print("ms  BPM="); Serial.print(60000.0 / rr, 1);

          // ── RMSSD calculation ───────────────────────────────────────────────
          // RMSSD = root mean square of successive RR differences.
          // Standard short-term HRV metric. Normal resting range: ~20–80ms.
          // Requires at least 5 beats to produce a meaningful value.
          if (rrCount >= 5) {
            int n = min(rrCount, RR_BUFFER_SIZE);  // samples in buffer
            double sumSqDiff = 0;
            int pairs = 0;

            for (int i = 1; i < n; i++) {
              // Successive difference between adjacent RR intervals
              long diff = rrBuffer[(rrCount - n + i) % RR_BUFFER_SIZE]
                        - rrBuffer[(rrCount - n + i - 1) % RR_BUFFER_SIZE];
              sumSqDiff += (double)diff * diff;
              pairs++;
            }

            double rmssd = sqrt(sumSqDiff / pairs);
            Serial.print("  RMSSD="); Serial.print(rmssd, 1); Serial.print("ms");
          }

          Serial.println();
        }

        lastPeakTime = now;
      }
    } else {
      // Signal dropped below threshold — reset rising flag for next peak
      rising = false;
    }

    particleSensor.nextSample();  // advance the library's FIFO read pointer
  }
}