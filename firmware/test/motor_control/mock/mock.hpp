#include<iostream>
#include <stdlib.h>     /* srand, rand */
#include <map>

// Mock definitions

class ServoInvertible
{
    private:
        int m_id;
        int m_pin;

    public:
        ServoInvertible();
        void writeMicroseconds(uint32_t us, int times);
        void attach(int pin);

};

void digitalWrite(int pin, int data);
int digitalRead(int pin);
int analogRead(int pin);

void delay(int us);

class SerialClass
{
    public:
        SerialClass();
        int available();
        void readBytes(char *buffer, int size);
};

// Checker
void checker(int level, uint32_t us);

// Definitions


void decode_thruster_data(uint32_t msg_data);
void error_loop();
void motorControlCallback (uint32_t msg_data);
void setup();
void loop(SerialClass Serial);