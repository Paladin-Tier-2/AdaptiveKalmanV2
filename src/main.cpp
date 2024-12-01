  #include <Arduino.h>
  #include <WiFi.h>
  #include <HTTPClient.h>
  #include <ArduiKalman.h>
 

  #define MAXDISTANCE 400
  #define WINDOW 80
  #define ALPHA 0.6f  // Forgetting factor 

  int stateNum = 1;
  int measureNum = 1;

  const char* ssid = "EkoEko";
  const char* password = "EkoEko2024";
  const char* serverUrl = "http://145.126.4.77:5000/submit";

  // Define the connections to the ultrasonic sensor
  const int trigPin = 14; // Use IO14 for the trigger pin, corresponds to Pin 13 on the ESP32 module. Use GPIO14 as the trigger pin
  const int echoPin = 27; // Use IO27 for the echo pin, corresponds to Pin 12 on the ESP32 module. Use GPIO27 as the echo pin

  long duration;
  int distance;
  int averageDistance;
  int element_counter;

  int previousDistance = MAXDISTANCE;


  float xc[1];        // correct state vector  -- x_k
  float xp[1];        // predict state vector --- Priori Estimate: x_k-1
  float A[1][1];      // prediction error covariance  --- F in the paper: Transistion matrix 
  float Q[1][1];      // process noise covariance -- Q  -- State Covaraince
  float R[1][1];      // measurement error covariance -- R -- 
  float H[1][1];      // Measurement model -- H
  float P[1][1];      // Post-prediction, pre-update // It is both apparently

  KalmanFilter m_kf;


// Assuming the expected value of the measurement process is zero.
float updateQ(float currentQ,float innovation, float alpha, float K) {
    float innovation_sq = innovation * innovation;

   float newQ = alpha * currentQ  + ( 1- alpha) *(K*innovation_sq*K);
    return newQ;
}

float updateR(float currentR, float residual, float alpha, float H,float P) {
    static float residualSum = 0.0;   

    // Update sum of residuals
    residualSum  = (residual * residual); // Sum of squared residuals

    // Calculate new R using a forgetting factor
    float newR = alpha * currentR + (1 - alpha) * (residualSum + H*P*H );

    return newR;
}


  void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
          delay(1000);
          Serial.println("Connecting to WiFi..");
      }
      
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
    pinMode(echoPin, INPUT);  // Sets the echoPin as an INPUT

    m_kf.init(stateNum, measureNum, &A[0][0], &P[0][0], &Q[0][0], &H[0][0], &R[0][0], &xp[0], &xc[0]);
    m_kf.zeros();
    A[0][0] = 1.0f;
    H[0][0] = 1.0f;
    Q[0][0] = 0.01f; // Value from paper
    R[0][0] = 0.1f; // Value from paper
    P[0][0] = 1.0f;

  element_counter = 0;
  }

  void loop() {
    // Clear the trigPin condition
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    // Set the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read the echoPin, return the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);

    // Calculating the distance
    distance= duration*0.034/2;  // Speed of sound wave divided by 2 (go and return)

      // Check if the distance is over 4 meters
    if (distance > MAXDISTANCE) {
      distance = previousDistance; // Use the last valid distance if the current is over 4m
    } 
    else {
      previousDistance = distance; // Update the last valid distance
    }
        
  // Print the distance on
    Serial.print( "Distance[cm]:");
    Serial.println(distance);
    delay(60);


  // How long does Kalman Take
 // float t1 = micros();

  // Kalman Filter
  float *predict = m_kf.predict();
  
  float innovation = distance - *predict; // python paper call it residual
  float measurement[measureNum];
  float K = (predict[0])*(H[0][0])* pow( (H[0][0]*(predict[0])*(H[0][0])+R[0][0]),-1 );
  measurement[0] = float(distance); 
  float *correct = m_kf.correct(measurement);
  float estimated_value = correct[0];

  float residual = measurement[0] - estimated_value; // Calculate residual

    R[0][0] = updateR(R[0][0], residual,ALPHA,H[0][0],P[0][0]); // Update R adaptively
    Q[0][0] = updateQ(Q[0][0], innovation,ALPHA,K); // Update Q adaptively
    m_kf.changeR(&R[0][0]);
    m_kf.changeQ(&Q[0][0]);



  if (WiFi.status() == WL_CONNECTED) {
      // Initialize HTTP client
      HTTPClient http;
      http.begin(serverUrl);
      http.addHeader("Content-Type", "application/json");
    // String httpRequestData = "{\"raw_distance\": " + String(distance) + ", \"average_distance\": " + String(estimated_value) + ", \"r_value\": " + String(R[0][0])+ "}"; 
     String httpRequestData = "{\"raw_distance\": " + String(distance) + ", \"average_distance\": " + String(estimated_value) + ", \"r_value\": " + String(R[0][0])+ ", \"q_value\": " + String(Q[0][0])+ "}"; 
      // Send HTTP POST request
      int httpResponseCode = http.POST(httpRequestData);
      
      // Print HTTP response code to Serial Monitor
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      
      // Free resources
      http.end();
    } else {
      Serial.println("WiFi not connected");
    }

  }
