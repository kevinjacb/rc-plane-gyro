// pins for reciever and control surfaces

#define RECVR_PIN 2
#define AILERON 3
#define ELEVATOR 5
#define THROTTLE 6
#define RUDDER 9

#include <Servo.h>
#include "MPU9250.h"
#include <Wire.h>

Servo aileron, elevator, throttle, rudder;
MPU9250 mpu;

const float Kp = 3, Ki = 0.1, Kd = 1; // PID constants

bool isEnabled = false;

float yaw,
    pitch, roll, prevYaw, prevPitch, prevRoll;
uint8_t aileronPID, elevatorPID, rudderPID;

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
unsigned int channels[10] = {1000}; // Note:  ppm can only use 8 channels
int c_channel = -1;

unsigned int out[10] = {1000};

volatile unsigned long last_time;

void readPPM()
{
    int pulse_width = micros() - last_time;
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

    pinMode(RECVR_PIN, INPUT_PULLUP); // set pin 2 as input (receiver

    attachInterrupt(digitalPinToInterrupt(RECVR_PIN), readPPM, RISING);

    aileron.attach(AILERON);
    elevator.attach(ELEVATOR);
    throttle.attach(THROTTLE);
    rudder.attach(RUDDER);

    last_time = micros();

    mpu.setup(0x68);
}

long last_print = 0, last_write = 0;

void loop()
{
    if (millis() - last_print > 1000)
    {
        last_print = millis();
        printChannels();
    }
    float desiredYaw, desiredPitch, desiredRoll;

    if (millis() - last_write > 10)
    {
        updateOutput();
        last_write = millis();
    }
    getAngles();
    mapChannels(desiredRoll, desiredPitch, desiredYaw);
    calculatePID(desiredRoll, desiredPitch, desiredYaw);
    out[2] = channels[2]; // no change to throttle, just pass through
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
    for (int i = 0; i < 10; i++)
    {
        Serial.print(out[i]);
        Serial.print(" ");
    }
    Serial.print("Is enabled: ");
    Serial.println(isEnabled);
    Serial.print("pid: ");
    Serial.println(aileronPID);
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

    aileronPID = rollError * Kp + (roll - prevRoll) * Kd + (rollError * Ki);
    elevatorPID = pitchError * Kp + (pitch - prevPitch) * Kd + (pitchError * Ki);
    // rudderPID = yawError * Kp + (yaw - prevYaw) * Kd + (yawError * Ki); // yaw pid disabled for now

    // constrain PID values
    // aileronPID = constrain(aileronPID, 1000, 2000);
    // elevatorPID = constrain(elevatorPID, 1000, 2000);
    // rudderPID = constrain(rudderPID, 1000, 2000); // yaw pid disabled for now

    // write PID values to output
    out[0] = channels[0] + ((isEnabled) ? aileronPID : 0);
    out[1] = channels[1] + ((isEnabled) ? elevatorPID : 0);
    out[3] = channels[3]; //+ rudderPID; // yaw pid disabled for now
}

void mapChannels(float &desiredRoll, float &desiredPitch, float &desiredYaw)
{
    // map channels to control surfaces

    desiredRoll = map(channels[0], 1000, 2000, -80, 80);
    desiredPitch = map(channels[1], 1000, 2000, -60, 90);
    desiredYaw = map(channels[3], 1000, 2000, -90, 90);

    isEnabled = (channels[4] > 1500) ? true : false;
}