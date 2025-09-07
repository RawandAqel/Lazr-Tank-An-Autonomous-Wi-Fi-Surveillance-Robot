#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char* ssid = "Rawand";
const char* password = "Rawand12345";



ESP8266WebServer server(80);

// Pins for motors
const int in1 = D1;
const int in2 = D2;
const int in3 = D3;
const int in4 = D4;

// IR sensors
const int irFront = D6;
const int irRight = D5;
const int irLeft = D7;

// LDR sensor and LED
//const int ldrPin = A0;
//const int ledPin = D8;

bool autoMode = false;

//turn off while tracking face
bool faceDetected = false;
unsigned long faceLostTime = 0;


void stopMotors() {
  Serial.println("stopMotors");
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void forward() {
  Serial.println("forward");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void backward() {
  Serial.println("backward");
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void left() {
  Serial.println("left");
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void right() {
  Serial.println("right");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}




void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.println("Connecting...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(irFront, INPUT);
  pinMode(irRight, INPUT);
  pinMode(irLeft, INPUT);

//  pinMode(ledPin, OUTPUT); // ← LED
//  pinMode(ldrPin, INPUT);  // ← LDR

  server.on("/", []() {
    server.send(200, "text/plain", "Tank is ready. Use /F /B /L /R /S /auto /manual");
  });

  server.on("/F", []() {
    if (!autoMode) forward();
    server.send(200, "text/plain", "Forward");
  });

  server.on("/B", []() {
    if (!autoMode) backward();
    server.send(200, "text/plain", "Backward");
  });

  server.on("/L", []() {
    if (!autoMode) left();
    server.send(200, "text/plain", "Left");
  });

  server.on("/R", []() {
    if (!autoMode) right();
    server.send(200, "text/plain", "Right");
  });

  server.on("/S", []() {
    if (!autoMode) stopMotors();
    server.send(200, "text/plain", "Stop");
  });

  server.on("/auto", []() {
    autoMode = true;
    server.send(200, "text/plain", "Auto mode activated");
  });

  server.on("/manual", []() {
    autoMode = false;
    stopMotors();
    server.send(200, "text/plain", "Manual mode activated");
  });

  server.on("/face_detected", []() {
    faceDetected = true;
    server.send(200, "text/plain", "Face detection received");
    Serial.println("face_detected");
  });

  server.on("/face_lost", []() {
    faceDetected = false;
    faceLostTime = millis(); 
    server.send(200, "text/plain", "Face lost");
    Serial.println("face_lost");
  });


  server.begin();
}

void loop() {
  server.handleClient();


  
  if (autoMode) {
    int front = digitalRead(irFront);
    int rightIR = digitalRead(irRight);
    int leftIR = digitalRead(irLeft);
//    Serial.println(front);
//    Serial.println(rightIR);
//    Serial.println(leftIR);
//    delay(2000);
    
    static unsigned long lastRead_automode = 0;
    unsigned long now_automode = millis();
    if (now_automode - lastRead_automode >= 5000){
      lastRead_automode = now_automode;
      stopMotors();
      delay(3000);
    }else{
      if(faceDetected) {
        stopMotors(); 
        delay(2000);
      }else{
        if (front == HIGH) {
          forward();
        }else if (leftIR == HIGH) {
          left();
          delay(1200);
        }else if (rightIR == HIGH) {
          right();
          delay(1200);
        } else {
          backward();
          delay(1500);
          stopMotors();
        }
      }
    }
    
    
    
  }
}
