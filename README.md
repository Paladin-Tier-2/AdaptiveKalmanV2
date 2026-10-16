# Adaptive Kalman Filter for Ultrasonic Sensor (ESP32)

An implementation of a 1D Adaptive Kalman Filter on an ESP32 to smooth distance measurements from an HC-SR04 ultrasonic sensor. The filter dynamically adjusts the measurement noise covariance ($R$) and process noise covariance ($Q$) at runtime using innovations and residuals, based on the method by [Akhlaghi et al. (2017)](https://www.researchgate.net/publication/313365845_Adaptive_Adjustment_of_Noise_Covariance_in_Kalman_Filter_for_Dynamic_State_Estimation).

The matrix and filter implementation is adapted from [TinyEKF](https://github.com/simondlevy/TinyEKF) by Simon D. Levy.

---

## Repository Structure

* `src/main.cpp` — ESP32 firmware: sensor polling, filter execution, and HTTP POST transmission.
* `src/ArduiKalman.h` & `src/ArduiKalman.cpp` — Adaptive Kalman Filter class implementation.
* `src/mat.h` — Matrix operations header.
* `platformio.ini` — PlatformIO build and board configuration.
* `server4.py` — Flask server that receives ESP32 telemetry over Wi-Fi and logs it to `measurements.csv`.
* `WiFiDistanceData3.m` — MATLAB script for plotting logged and live distance data.

---

## Hardware Setup

| HC-SR04 Pin | ESP32 Pin | Notes |
| :--- | :--- | :--- |
| **VCC** | 5V (`VIN`) | Drawn directly from the USB 5V rail via the onboard protection diode. |
| **Trig** | GPIO 14 | 3.3V logic output is sufficient to trigger the HC-SR04. |
| **Echo** | GPIO 27 | Outputs 5V logic; use a voltage divider or level shifter to step down to 3.3V. |
| **GND** | GND | Common ground. |

---

## Usage

### 1. Flash the ESP32
1. Update the Wi-Fi credentials and target server IP in `src/main.cpp`:
   ```cpp
   const char* ssid = "Your_SSID";
   const char* password = "Your_Password";
   const char* serverUrl = "http://your.server.ip:5000/submit";
   ```
2. Build and upload using PlatformIO:
   ```bash
   pio run --target upload
   ```

### 2. Run the Telemetry Server
Wait for the ESP32 to connect to the Wi-Fi network (verify via the serial monitor or onboard LED) before starting the server so initial packets are not dropped.

```bash
pip install flask
python server4.py
```

### 3. Plot the Data
Open MATLAB in the project root directory and run `WiFiDistanceData3.m` to visualize raw vs. filtered distance measurements from `measurements.csv`.

---

## Mathematical Formulation

For a system with $n$ states and $m$ measurements:
* $\hat{x}_{k|k-1}, \hat{x}_{k|k} \in \mathbb{R}^n$: *A priori* (predicted) and *a posteriori* (corrected) state vectors (`xp` and `xc` in code).
* $A \in \mathbb{R}^{n \times n}$: State transition matrix.
* $H \in \mathbb{R}^{m \times n}$: Observation matrix.
* $P \in \mathbb{R}^{n \times n}$: Estimate error covariance matrix.
* $Q \in \mathbb{R}^{n \times n}$: Process noise covariance matrix.
* $R \in \mathbb{R}^{m \times m}$: Measurement noise covariance matrix.
* $z_k \in \mathbb{R}^m$: Sensor measurement at step $k$.

### 1. Prediction Step
$$\hat{x}_{k|k-1} = A \hat{x}_{k-1|k-1}$$

$$P_{k|k-1} = A P_{k-1|k-1} A^T + Q_{k-1}$$

### 2. Correction Step
$$K_k = P_{k|k-1} H^T \left( H P_{k|k-1} H^T + R_{k-1} \right)^{-1}$$

$$\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k \left( z_k - H \hat{x}_{k|k-1} \right)$$

$$P_{k|k} = \left( I - K_k H \right) P_{k|k-1}$$

### 3. Adaptive Covariance Update
Using the innovation (pre-fit residual) $d_k = z_k - H \hat{x}_{k|k-1}$, the measurement residual (post-fit residual) $\epsilon_k = z_k - H \hat{x}_{k|k}$, and a forgetting factor $\alpha \in (0, 1)$:

$$R_k = \alpha R_{k-1} + (1 - \alpha) \left( \epsilon_k \epsilon_k^T + H P_{k|k-1} H^T \right)$$

$$Q_k = \alpha Q_{k-1} + (1 - \alpha) \left( K_k d_k d_k^T K_k^T \right)$$

### 1D System Initialization
Because distance is tracked as a 1st-order constant-position model ($n = m = 1$), $A = 1$ and $H = 1$. In `src/main.cpp`, the matrices are initialized as:

```cpp
int stateNum = 1;
int measureNum = 1;

float xc[1], xp[1];
float A[1][1] = {{1.0f}};
float H[1][1] = {{1.0f}};
float Q[1][1] = {{0.01f}};
float R[1][1] = {{0.1f}};
float P[1][1] = {{1.0f}};
```

---

## Reference

* S. Akhlaghi, N. Zhou, and Z. Huang, "Adaptive adjustment of noise covariance in Kalman filter for dynamic state estimation," *2017 IEEE Power & Energy Society General Meeting*, Chicago, IL, USA, 2017, pp. 1–5.
