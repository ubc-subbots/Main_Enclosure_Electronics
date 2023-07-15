#include "mock.hpp"

#include "../motor_control.cpp"

int servo_id = 0;
int digitalReadPin = 0;
bool serial_available = false;
bool randomize = false;
int check_count = 0;

uint32_t cur_pwm = 0;

std::map<int, int> pin_to_servo;

std::map<int, uint32_t> ref_level_to_us;

ServoInvertible::ServoInvertible()
{
    this->m_id = servo_id++;
}

void ServoInvertible::writeMicroseconds(uint32_t us, int times) 
{
    std::cout << "Servo " << this->m_id << " at pin " << this->m_pin << " is written " << us << " us";
    std::cout << " for level: " << ((cur_pwm >> (this->m_id * 5)) & 0x1f) << std::endl;
    checker(((cur_pwm >> (this->m_id * 5)) & 0x1f), us);
}

void ServoInvertible::attach(int pin) 
{
    pin_to_servo[pin] = this->m_id;
    this->m_pin = pin;
}

void digitalWrite(int pin, int data) 
{
    std::cout << "Pin " << pin << " is written " << data << std::endl;
    // Only this pin is read from /written to. 
    digitalReadPin = data;
}

int digitalRead(int pin)
{
    std::cout << "Pin " << pin << " is read " << digitalReadPin << std::endl;
    return digitalReadPin;
}

int analogRead(int pin)
{
    std::cout << "Analog Pin " << pin << " is read " << 300 << std::endl;
    return 300;
}

void delay(int us)
{
    std::cout << "Delay requested for " << us << "us" << std::endl;
    // We don't want to delay when testing
    return;
}

SerialClass::SerialClass()
{

}

int SerialClass::available()
{
    serial_available = !serial_available;
    if (serial_available)
    {
        std::cout << "Serial available" << std::endl;
        return 4;
    }
    else
    {
        std::cout << "Serial not available" << std::endl;
        return 0;
    }
}

int isolate_helper(uint32_t data, int level)
{
    return ((data >> (level * 5)) & 0x1f);
}


void SerialClass::readBytes(char *buffer, int size)
{
    if (randomize)
    {
        cur_pwm = std::rand();
    }
    else 
    {
        cur_pwm += 1 + (1 << 5) + (1 << 10) + (1 << 15) + (1 << 20);
        if (cur_pwm > (1 << 25)) 
        {
            cur_pwm = 0;
        }
    }
    std::cout << "Serial read bytes " << cur_pwm << std::endl;
    std::cout << "Serial read levels " << std::endl;
    std::cout << "    1: " << isolate_helper(cur_pwm, 0) << std::endl;
    std::cout << "    2: " << isolate_helper(cur_pwm, 1) << std::endl;
    std::cout << "    3: " << isolate_helper(cur_pwm, 2) << std::endl;
    std::cout << "    4: " << isolate_helper(cur_pwm, 3) << std::endl;
    std::cout << "    5: " << isolate_helper(cur_pwm, 4) << std::endl;
    *buffer = cur_pwm & 0xff;
    *(buffer+1) = (cur_pwm >> 8) & 0xff;
    *(buffer+2) = (cur_pwm >> 16) & 0xff;
    *(buffer+3) = (cur_pwm >> 24) & 0xff;
}


void setup_checker() 
{
    // Hardcode, to avoid checking values with the same 
    // calculation procedure as the code under testing. 
    ref_level_to_us[0] = B_MAX_US;
    ref_level_to_us[1] = B_MAX_US + B_FACTOR * 1;
    ref_level_to_us[2] = B_MAX_US + B_FACTOR * 2;
    ref_level_to_us[3] = B_MAX_US + B_FACTOR * 3;
    ref_level_to_us[4] = B_MAX_US + B_FACTOR * 4;
    ref_level_to_us[5] = B_MAX_US + B_FACTOR * 5;
    ref_level_to_us[6] = B_MAX_US + B_FACTOR * 6;
    ref_level_to_us[7] = B_MAX_US + B_FACTOR * 7;
    ref_level_to_us[8] = B_MAX_US + B_FACTOR * 8;
    ref_level_to_us[9] = B_MAX_US + B_FACTOR * 9;
    ref_level_to_us[10] = B_MAX_US + B_FACTOR * 10;
    ref_level_to_us[11] = B_MAX_US + B_FACTOR * 11;
    ref_level_to_us[12] = B_MAX_US + B_FACTOR * 12;
    ref_level_to_us[13] = B_MAX_US + B_FACTOR * 13;
    ref_level_to_us[14] = B_MAX_US + B_FACTOR * 14;
    ref_level_to_us[15] = B_MAX_US + B_FACTOR * 15;
    ref_level_to_us[16] = STATIONARY_US;
    ref_level_to_us[17] = F_MIN_US;
    ref_level_to_us[18] = F_MIN_US + F_FACTOR * 1;
    ref_level_to_us[19] = F_MIN_US + F_FACTOR * 2;
    ref_level_to_us[20] = F_MIN_US + F_FACTOR * 3;
    ref_level_to_us[21] = F_MIN_US + F_FACTOR * 4;
    ref_level_to_us[22] = F_MIN_US + F_FACTOR * 5;
    ref_level_to_us[23] = F_MIN_US + F_FACTOR * 6;
    ref_level_to_us[24] = F_MIN_US + F_FACTOR * 7;
    ref_level_to_us[25] = F_MIN_US + F_FACTOR * 8;
    ref_level_to_us[26] = F_MIN_US + F_FACTOR * 9;
    ref_level_to_us[27] = F_MIN_US + F_FACTOR * 10;
    ref_level_to_us[28] = F_MIN_US + F_FACTOR * 11;
    ref_level_to_us[29] = F_MIN_US + F_FACTOR * 12;
    ref_level_to_us[30] = F_MIN_US + F_FACTOR * 13;
    ref_level_to_us[31] = F_MIN_US + F_FACTOR * 14;
}

void checker(int level, uint32_t us) 
{
    if (check_count > 5)
    {
        uint32_t ref_us = ref_level_to_us[level];

        if (ref_us != us) 
        {
            std::cout << "ERROR: set " << us << " != expected " << ref_us << std::endl;
        }
        else
        {
            std::cout << "INFO: set " << us << " == expected " << ref_us << std::endl;
        }
    }
    check_count ++;

}


int main(void) 
{
    setup_checker();

    setup();
    SerialClass Serial;

    // Bring up test

    // we care about the first 5 thrusters
    for (int i = 0; i < 32; i++)
    {
        loop(Serial); // TODO: figure out how to omit loop argument
    }

    // Exploration test
    randomize = true;
    for (int i = 0; i < 100; i++)
    {
        loop(Serial); 
    }

    std::cout << "Test done!" << std::endl;

    return 0;
}