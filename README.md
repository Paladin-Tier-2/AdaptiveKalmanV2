
# Adaptive Kalman Filter for Ultrasonic Sensor with ESP32

## 📜 Description

This repository contains an implementation of an **Adaptive Kalman Filter** for smoothing sensor readings from an **HC-SR04 Ultrasonic Sensor**. The system uses an **ESP32 microcontroller** and dynamically adjusts the Kalman filter's noise covariance matrices \( R \) and \( Q \) based on innovations and residuals, inspired by the paper:

- Akhlaghi, N., Zhou, and Z. Huang, _"Adaptive adjustment of noise covariance in Kalman filter for dynamic state estimation,"_ in **2017 IEEE Power & Energy Society General Meeting**.

---

## 🛠️ Hardware & Tools

### Hardware Components

- **ESP32 Microcontroller**
- **HC-SR04 Ultrasonic Sensor**
- **USB Boost Converter** (for powering ESP32)
- **Breadboard** and **Jumper Wires**

### Software & Frameworks

- **PlatformIO IDE** or **Arduino IDE**
- **Arduino Core for ESP32**
- **Modified Kalmnan Filter library** (based on [TinkEKY by **simondlevy**][https://github.com/simondlevy/TinyEKF]
- **Python 3** (for Flask server)
- **MATLAB** (for live data plotting)

---

## 🧩 Features

- **Real-Time Adaptive Tuning** of \( R \) and \( Q \) for accurate and smooth data.
- **Wi-Fi Connectivity** using ESP32 for remote data transmission.
- **Flask Server Integration** for data logging and analysis.
- **Live Data Plotting** using MATLAB.
- **First-Order System Implementation** for simplicity and efficiency.

---

## 📂 Directory Structure

```
├── src/
│   ├── main.cpp             # Main code for the ESP32 implementation
│   ├── ArduiKalman.h        # Header file for Kalman Filter implementation
│   ├── ArduiKalman.cpp      # Source file for Kalman Filter implementation
│   ├── mat.h                # Matrix operations header
├── include/
│   ├── platformio.ini       # PlatformIO configuration file
├── flask_server/
│   ├── server.py            # Python Flask server for data collection
│   ├── measurements.csv     # CSV file to store incoming data
├── matlab/
│   ├── live_plot.m          # MATLAB script for live data plotting
├── README.md                # This README file
```

---

## ⚙️ Setup Instructions

### 1. Clone the Repository

```bash
git clone https://github.com/your-username/esp32-adaptive-kalman.git
cd esp32-adaptive-kalman
```

### 2. Install Dependencies

- Install **PlatformIO**: [PlatformIO Installation Guide](https://platformio.org/install)  
- Add libraries:
  - **Arduino Core for ESP32**

### 3. Hardware Setup

Connect the **HC-SR04** Ultrasonic Sensor to the **ESP32** as follows:

- **Trig Pin**: Connect to **GPIO 14** on ESP32
- **Echo Pin**: Connect to **GPIO 27** on ESP32
- **VCC**: Connect to **5V** (use a level shifter if necessary)
- **GND**: Connect to **GND**

### 4. Configure Wi-Fi Credentials

In `main.cpp`, update the following lines with your Wi-Fi network credentials:

```cpp
const char* ssid = "Your_SSID";
const char* password = "Your_Password";
const char* serverUrl = "http://your.server.ip:5000/submit";
```

### 5. Build and Upload Code to ESP32

- Open the project in **PlatformIO IDE** or **Arduino IDE**.
- Configure your `platformio.ini` file with the correct board settings for your ESP32.
- Connect your ESP32 to your computer and upload the code:

```bash
pio run --target upload
```

### 6. Start the Flask Server

**Important:** After uploading the code to the ESP32, wait until the ESP32 connects to your Wi-Fi network (indicated by the blinking LED stopping or a serial message). Once the ESP32 is connected:

- Navigate to the `flask_server/` directory:

  ```bash
  cd flask_server
  ```

- Install Python dependencies:

  ```bash
  pip install flask
  ```

- Start the Flask server:

  ```bash
  python server.py
  ```

### 7.  📊 Plotting via MATLAB

To visualize the data

- Open MATLAB.
- Navigate to the `matlab/` directory.
- Run the `WiFiDistanceData3_publish.m` script.
---

## 🌐 Python Flask Server

### Flask Server Code (`server.py`)


#### Explanation

- **Purpose:** Receives data from the ESP32 via HTTP POST requests and logs it to a CSV file (`measurements.csv`).
- **Data Validation:** Ensures all required fields are present in the incoming JSON data.
- **Data Storage:** Appends the received data to `measurements.csv` for later analysis or live plotting.

#### Instructions

- Ensure Python 3 is installed.
- Install Flask by running `pip install flask`.
- Run the server using `python server.py`.

---

## 🧪 Experiments & Results

### Prototype Setup

A battery-powered test setup was built to simulate real-world scenarios. The ESP32 transmits sensor readings wirelessly to the Flask server.

### Filter Performance

### Observations

- The filter accurately filters out undesired peaks and flattens out the noisy measurements.
- The adaptive adjustment of \( R \) and \( Q \) allows the filter to respond to changes in measurement noise dynamically.

---

## 📚 References

1. **Paper:**
   - Akhlaghi, N., Zhou, Z., and Huang, Z., "Adaptive adjustment of noise covariance in Kalman filter for dynamic state estimation," in *2017 IEEE Power & Energy Society General Meeting*. [Paper](https://www.researchgate.net/publication/313365845_Adaptive_Adjustment_of_Noise_Covariance_in_Kalman_Filter_for_Dynamic_State_Estimation)

---


### Kalman Filter Guide

The Kalman Filter can be thought of as
a black box where it takes noisy inputs and makes a prediction
and outputs it. The estimations are the hidden variables - the state
variables - that may or may not be measured. It is a great estimator
of the state variables. Ideally, a Kalman filter can track the hidden
system parameters. The Kalman can be used as it is a general
algorithm that can be easily implemented into any system. It has
heavy documentation and already-made implementations because of
the reliability and power of the filter.
It can be thought of as a two-step filter where one step is based
on system dynamics, while the latter is the sensor inputs and it all
merged with the statistical properties of the system and gain of the
filter.


- **n**: Number of **states** in the system.
- **m**: Number of **measurement values**.

#### Kalman Filter Equations

# Kalman Filter Documentation

## Variables and Matrices
- **xc[n]:** Corrected (updated) state vector at time \( k \).  
  The estimated state after considering the measurement.
- **xp[n]:** Predicted state vector at time \( k \).  
  The state prediction based on the previous state and the system model.
- **A[n][n]:** System dynamics matrix.  
  Describes how the system evolves from one state to the next without considering the process noise or control input.
- **H[m][n]:** Measurement matrix.  
  State-to-Measurement matrix 
- **Q[n][n]:** Process noise covariance matrix.  
  The covariance of the process noise, accounting for uncertainties in the system model.
- **R[m][m]:** Measurement noise covariance matrix.  
  The covariance of the measurement noise of uncertainties in the sensor measurements.
- **P[n][n]:** Estimate error covariance matrix.  
  Represents the error covariance in the state estimate.

## Kalman Filter Equations


### 1. Prediction Step:
- **State Prediction:**  
```math
  \hat{x}_{k|k-1} = A \hat{x}_{k-1|k-1}
```
- **Covariance Prediction:**  
```math
  P_{k|k-1} = A P_{k-1|k-1} A^T + Q
```

## 2. Update (Correction) Step:
- **Kalman Gain Calculation:**  
```math
  K_k = P_{k|k-1} H^T \left( H P_{k|k-1} H^T + R \right)^{-1}
  ```
- **State Update:**  
```math
  \hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k \left( z_k - H \hat{x}_{k|k-1} \right)
  ```
- **Covariance Update:**  
```math
  P_{k|k} = \left( I - K_k H \right) P_{k|k-1}
  ```

## Adaptive Adjustment Equations

- **Innovation (Predicted Residual):**  
```math
  d_k = z_k - H \hat{x}_{k|k-1}
  ```
- **Residual (Measurement Residual):**  
```math
  \epsilon_k = z_k - H \hat{x}_{k|k}
 ```
- **Adaptive Update of \( R \):**  
 ```math
  R_k = \alpha R_{k-1} + (1 - \alpha) \left( \epsilon_k \epsilon_k^T + H P_{k|k-1} H^T \right)
  ```
- **Adaptive Update of \( Q \):**  
  ```math
  Q_k = \alpha Q_{k-1} + (1 - \alpha) \left( K_k d_k d_k^T K_k^T \right)
  ```

## Parameters

- **\( \alpha \):** Forgetting factor (\( 0 < \alpha < 1 \)).  
  Controls the weight given to new measurements versus the existing estimate.
- **\( K_k \):** Kalman Gain at time \( k \).  
  Determines how much the predictions are corrected based on the new measurement.
- **\( z_k \):** Measurement at time \( k \).


## Implementation Notes

In this project:
- We use a first-order system, so \( n = m = 1 \).
- The state vector \( x \) represents the distance measured by the ultrasonic sensor.
- The system dynamics matrix \( A \) is set to 1, assuming a constant system without any control input.
- The measurement matrix \( H \) is also set to 1, directly mapping the state to the measurement.
- The process and measurement noise covariances \( Q \) and \( R \) are initialized with small positive values and are updated adaptively.


### Code Snippet for Variable Initialization

In `main.cpp`, the Kalman filter variables are initialized as follows:

```cpp
int stateNum = 1;     // Number of states (n)
int measureNum = 1;   // Number of measurements (m)

float xc[1];          // Corrected state vector
float xp[1];          // Predicted state vector
float A[1][1];        // System dynamics matrix
float Q[1][1];        // Process noise covariance
float R[1][1];        // Measurement noise covariance
float H[1][1];        // Measurement matrix
float P[1][1];        // Estimate error covariance

// Initialization
A[0][0] = 1.0f;       // Assuming a constant system
H[0][0] = 1.0f;       // Direct measurement
Q[0][0] = 0.01f;      // Initial process noise covariance
R[0][0] = 0.1f;       // Initial measurement noise covariance
P[0][0] = 1.0f;       // Initial estimate error covariance
```

---


**Note:** It's important to start the Flask server **after** the ESP32 is connected to the Wi-Fi hotspot created by your laptop. This ensures that the server is ready to receive data, and there are no connection issues.

# Important Tips

- **Hotspot Connection:** When testing, ensure your laptop's hotspot is active and the ESP32 is configured to connect to it.
- **Serial Monitoring:** Use a serial monitor to observe the ESP32's status messages.
- **Data Synchronization:** Starting the server after the ESP32 is connected prevents data loss and ensures all measurements are logged.
- **5V** If you look at the schematic of the ESP32, you can see if ESP32 is powered by a 5V source through the USB, then the $`V_in`$ is directly connected to the USB port through a diode, th


---
