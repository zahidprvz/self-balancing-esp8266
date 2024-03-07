//#define AP_MODE 1
#define STA_MODE 1

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

#ifdef STA_MODE
    #include <DNSServer.h>
    #include <WiFiManager.h>      
#endif


float Kp = 55;
float Kd = -0.5;
float Ki = 70;
float targetAngle = -0.35;

/*********HTML Contents************/
const char index_html[] PROGMEM = R"=====(
    <!DOCTYPE html>
    <html>
    <head>
        <title>Self-Balancing Robot</title>
        <script src="/js/zepto.min.js"></script>
    </head>
    <body>
        <h1>Self-Balancing Robot Control Panel</h1>
        <h2>Adjust Parameters:</h2>
        <form action="/Tune">
            <label for="angle">Target Angle:</label>
            <input type="text" id="angle" name="angle"><br><br>
            <label for="i">Ki:</label>
            <input type="text" id="i" name="i"><br><br>
            <label for="d">Kd:</label>
            <input type="text" id="d" name="d"><br><br>
            <label for="p">Kp:</label>
            <input type="text" id="p" name="p"><br><br>
            <input type="submit" value="Submit">
        </form>
        <h2>Current Parameters:</h2>
        <p id="parameters"></p>
        <script>
            function updateParameters() {
                var xhttp = new XMLHttpRequest();
                xhttp.onreadystatechange = function() {
                    if (this.readyState == 4 && this.status == 200) {
                        document.getElementById("parameters").innerHTML = this.responseText;
                    }
                };
                xhttp.open("GET", "/Tune?ok=1", true);
                xhttp.send();
            }
            setInterval(updateParameters, 1000);
        </script>
    </body>
    </html>
)=====";

/**********JS ***********/
const char jquery_js[] PROGMEM = R"=====(
    // Zepto.js library goes here
)=====";

void handlejQuery(){
    httpServer.send_P ( 200, "application/js", jquery_js);
}

void handleRoot() { 
    httpServer.send_P ( 200, "text/html", index_html);   
}

void handleTuning(){    
    // Handle parameter tuning here
}

void handleNotFound() {
    String message = "File Not Found\n\n";
    httpServer.send ( 404, "text/plain", message );    
}

#define MotorPWMPin1   D3
#define MotorPWMPin2   D4
#define MotorPWMPin3   D5 // Additional pin for second motor
#define MotorPWMPin4   D6 // Additional pin for second motor
#define sampleTime  0.01
MPU6050 mpu;
int16_t accY, accZ, gyroX;
volatile int motorPowerA, motorPowerB, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;

void setMotors(int MotorPWMA, int MotorPWMB) {
    if(MotorPWMA >= 0) {
        analogWrite(MotorPWMPin1, MotorPWMA);
        analogWrite(MotorPWMPin2, 0);
    } else {
        analogWrite(MotorPWMPin2, -MotorPWMA);
        analogWrite(MotorPWMPin1, 0);
    }
    if(MotorPWMB >= 0) {
        analogWrite(MotorPWMPin3, MotorPWMB);
        analogWrite(MotorPWMPin4, 0);
    } else {
        analogWrite(MotorPWMPin4, -MotorPWMB);
        analogWrite(MotorPWMPin3, 0);
    }
}

void setup() {
    pinMode(MotorPWMPin1, OUTPUT);
    pinMode(MotorPWMPin2, OUTPUT);
    pinMode(MotorPWMPin3, OUTPUT); // Additional pin for second motor
    pinMode(MotorPWMPin4, OUTPUT); // Additional pin for second motor
    setMotors(0, 0);
    Serial.begin(115200);
    Wire.begin();
    analogWriteRange(255);
    #ifdef STA_MODE
    WiFiManager wifiManager;
    wifiManager.autoConnect("RobotWifiConfig");  
    #endif
    mpu.initialize();
    mpu.setXAccelOffset(-553);
    mpu.setYAccelOffset(460);
    mpu.setZAccelOffset(1139);
    mpu.setXGyroOffset(2748);
    mpu.setYGyroOffset(-2254);
    mpu.setZGyroOffset(1063);  
    // initialize PID sampling loop
    httpUpdater.setup(&httpServer);
    httpServer.on("/", handleRoot); // Corrected route for root page
    httpServer.on("/Tune", handleTuning);
    httpServer.on(jquery_js, handlejQuery);      
    httpServer.onNotFound(handleNotFound); 
    httpServer.begin();
}


unsigned int lastTempUpdate=0;
unsigned int counter=0;
void periodicFunc() {
    if ((millis() - lastTempUpdate) > (sampleTime*1000)) {
        lastTempUpdate=millis();
        accAngle = atan2(accY, accZ)*RAD_TO_DEG;
        gyroRate = map(gyroX - 2748, -32768, 32767, -250, 250); // Adjusted gyroX with offset
        gyroAngle = (float)gyroRate*sampleTime;  
        currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);
        error = currentAngle - targetAngle;
        errorSum = errorSum + error;  
        errorSum = constrain(errorSum, -300, 300);
        motorPowerA = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
        motorPowerB = motorPowerA; // Same power for both motors, adjust as needed
        prevAngle = currentAngle;  
        motorPowerA = constrain(motorPowerA, -255, 255);
        motorPowerB = constrain(motorPowerB, -255, 255);
        setMotors(motorPowerA, motorPowerB); 
        counter++;
        if(counter%200==0)
        {
            Serial.print("Angle: ");
            Serial.println(currentAngle);   
            Serial.print("Output: ");
            Serial.println(motorPowerA);   
        }
    }
}

void loop() {
    httpServer.handleClient();
    accY = mpu.getAccelerationY();
    accZ = mpu.getAccelerationZ();  
    gyroX = mpu.getRotationX();
    periodicFunc();
}
