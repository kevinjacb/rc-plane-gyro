// pins for reciever and control surfaces

#define RECVR_PIN 2
#define AILERON 3
#define ELEVATOR 5
#define THROTTLE 6
#define RUDDER 9
#define BATTERYPIN A0

#include <Servo.h>
#include "MPU9250.h"
#include <Wire.h>
#include <EEPROM.h>

// #define DEBUG_EN 0 // enable debug mode
// #define DEBUG(x) Serial.println(x)
// #define BL_EN 1 // enable bluetooth mode

Servo aileron, elevator, throttle, rudder;
MPU9250 mpu;

float Kp = 10, Ki = 0, Kd = 3; // PID constants

byte isEnabled = 0;
byte testControlSurfaces = 0;
byte invertAileron = 0, invertElevator = 0, invertRudder = 0;
byte sendData = 1; // send data over bluetooth

float yaw,
    pitch, roll, prevYaw, prevPitch, prevRoll;
uint16_t aileronPID, elevatorPID, rudderPID;

byte writeToEEPROM = 0;
unsigned long lastWriteEEPROM = 0;

float batteryPercent = 0;

/*
    *  PPM receiver
    channel 1 - aileron
    channel 2 - elevator
    channel 3 - throttle
    channel 4 - rudder
    channel 5 - aux 1
    channel 6 - aux 2
    channel 7 - aux 3
    channel 8 - aux 4

    */

/* Battery voltage divider
    with 10k resistor and 4.7k resistor
    batt @ 12.6v output ~ 4.1v
    batt @ 10.5v output ~ 3.3v
*/

uint16_t channels[10] = {1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; // Note:  ppm can only use 8 channels
uint8_t c_channel = -1;

uint16_t out[10] = {1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

volatile unsigned long last_time;

void readPPM()
{
    uint16_t pulse_width = micros() - last_time;
    last_time = micros();

    if (c_channel == -1 || pulse_width > 4000)
    {
        if (pulse_width > 4000)
            c_channel = 0;
    }
    else
    {
        channels[c_channel] = pulse_width;
        if (c_channel == 9)
            c_channel = 0;
        else
            c_channel++;
    }
}

void setup()
{
    Serial.begin(115200); // initialize serial communication
    Wire.begin();

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(RECVR_PIN, INPUT_PULLUP); // set pin 2 as input (receiver)
    pinMode(BATTERYPIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(RECVR_PIN), readPPM, RISING);

    aileron.attach(AILERON);
    elevator.attach(ELEVATOR);
    throttle.attach(THROTTLE);
    rudder.attach(RUDDER);

    EEPROM.get(0, Kp);
    EEPROM.get(4, Ki);
    EEPROM.get(8, Kd);
    EEPROM.get(12, invertAileron);
    EEPROM.get(13, invertElevator);
    EEPROM.get(14, invertRudder);

    updateOutput();

    last_time = micros();

    // mpu setup
    mpu.setup(0x68);
    // change the following values to match your sensor
    mpu.setAccBias(753.27, -446.93, 1055.02);
    mpu.setGyroBias(41.95, 303.32, 113.12);
    mpu.setMagBias(184.57, 336.55, 24.76);
}

long last_write = 0;

void loop()
{
    // if (millis() - last_print > 1000)
    // {
    //     last_print = millis();
    //     printChannels();
    // }
    float desiredYaw, desiredPitch, desiredRoll;
    getAngles();
    rawMapChannels();
    mapChannels(desiredRoll, desiredPitch, desiredYaw);
    calculatePID(desiredRoll, desiredPitch, desiredYaw);
    getData();

    batteryPercent = map(analogRead(BATTERYPIN) * 0.0049, 3.3, 4.1, 0, 100); // convert battery voltage to percentage

    if (millis() - last_write > 100 && sendData && channels[2] < 1080)
    {
        last_write = millis();
        // need to save spaceee
        Serial.println("     ___" + String(Kp) + "," + String(Ki) + "," + String(Kd) + "," + String(pitch) + "," + String(roll) + "," + String(yaw) + "," + String(batteryPercent) + "___    ");
    }

    out[2] = channels[2]; // no change to throttle, just pass through

    updateOutput();
}

void updateOutput()
{
    aileron.writeMicroseconds(out[0]);
    elevator.writeMicroseconds(out[1]);
    throttle.writeMicroseconds(out[2]);
    rudder.writeMicroseconds(out[3]);
}

void printChannels()
{
    for (uint8_t i = 0; i < 10; i++)
    {
        Serial.print(out[i]);
        Serial.print(" ");
    }
    Serial.print("Is enabled: ");
    Serial.println(isEnabled);
    Serial.print("aileron pid: ");
    Serial.println(out[0]);
    Serial.print("Angles: ");
    Serial.print("yaw: ");
    Serial.print(yaw);
    Serial.print(" pitch: ");
    Serial.print(pitch);
    Serial.print(" roll: ");
    Serial.println(roll);
}

void getAngles() // get angles/direction from sensor
{
    if (mpu.update())
    {
        // Serial.print(mpu.getYaw());
        // Serial.print(", ");
        // Serial.print(mpu.getPitch());
        // Serial.print(", ");
        // Serial.println(mpu.getRoll());
        prevYaw = yaw;
        prevPitch = pitch;
        prevRoll = roll;

        yaw = mpu.getYaw();
        pitch = mpu.getPitch();
        roll = mpu.getRoll();
    }
}

// yaw pid disabled for now
void calculatePID(float desiredYaw, float desiredPitch, float desiredRoll)
{
    // calculate PID for each control surface
    float yawError = desiredYaw - yaw;
    float pitchError = desiredPitch - pitch;
    float rollError = desiredRoll - roll;

    //
    // #if DEBUG_EN
    //     DEBUG(String("Yaw_Err" + yawError));
    //     DEBUG(String("Pitch_Err" + pitchError));
    //     DEBUG(String("Roll_Err" + rollError));
    // #endif

    aileronPID = rollError * Kp + (roll - prevRoll) * Kd + (rollError * Ki);
    elevatorPID = pitchError * Kp + (pitch - prevPitch) * Kd + (pitchError * Ki);
    // rudderPID = yawError * Kp + (yaw - prevYaw) * Kd + (yawError * Ki); // yaw pid disabled for now

    // constrain PID values

    // write PID values to output
    out[0] = channels[0] + ((isEnabled) ? ((invertAileron) ? -aileronPID : aileronPID) : 0);
    out[1] = channels[1] + ((isEnabled) ? ((invertElevator) ? -elevatorPID : elevatorPID) : 0);
    out[3] = channels[3]; //+ rudderPID; // yaw pid disabled for now
    out[0] = constrain(out[0], 1000, 2000);
    out[1] = constrain(out[1], 1000, 2000);
    out[3] = constrain(out[3], 1000, 2000);
}

void rawMapChannels() // extend the coverage of the servos
{
    // map channels to control surfaces

    channels[0] = map(channels[0], 1000, 2000, 700, 2400);
    channels[1] = map(channels[1], 1000, 2000, 700, 2400);
    channels[3] = map(channels[3], 1000, 2000, 700, 2400);

    isEnabled = channels[4] > 1500;
}

void mapChannels(float &desiredRoll, float &desiredPitch, float &desiredYaw)
{
    // map channels to control surfaces

    desiredRoll = map(channels[0], 1000, 2000, -80, 80);
    desiredPitch = map(channels[1], 1000, 2000, -60, 90);
    desiredYaw = map(channels[3], 1000, 2000, -90, 90);
}

void getData()
{ // get command from app
    if (Serial.available() > 0)
    {
        String command = Serial.readStringUntil('\n');
        Serial.println(command);
        if (command == "set")
        {
            Kp = Serial.readStringUntil(',').toFloat();
            Ki = Serial.readStringUntil(',').toFloat();
            Kd = Serial.readStringUntil(',').toFloat();
            invertAileron = Serial.readStringUntil(',').toInt();
            invertElevator = Serial.readStringUntil(',').toInt();
            invertRudder = Serial.readStringUntil(',').toInt();
            testControlSurfaces = Serial.readStringUntil(',').toInt();
            writeToEEPROM = Serial.readStringUntil(',').toInt();
        }
        else if (command == "on")
            sendData = 1;
        else if (command == "off")
            sendData = 0;
        if (testControlSurfaces)
            selfTest();
        testControlSurfaces = 0;
        if (writeToEEPROM && millis() - lastWriteEEPROM > 60000)
        {
            writeEEPROM();
            writeToEEPROM = 0;
            lastWriteEEPROM = millis();
        }
    }
}

void selfTest() // test control surfaces
{
    for (uint8_t i = 0; i < 4; i++)
    {
        if (i == 2)
            continue;
        out[i] = 1000;
        updateOutput();
        delay(1000);
        out[i] = 2000;
        updateOutput();
        delay(1000);
        out[i] = 1500;
        updateOutput();
        delay(1000);
    }
}

void writeEEPROM() // write PID values to EEPROM
{
    digitalWrite(LED_BUILTIN, HIGH);
    EEPROM.put(0, Kp);
    EEPROM.put(4, Ki);
    EEPROM.put(8, Kd);
    EEPROM.put(12, invertAileron);
    EEPROM.put(13, invertElevator);
    EEPROM.put(14, invertRudder);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
}
