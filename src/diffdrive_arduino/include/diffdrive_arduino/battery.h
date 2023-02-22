#ifndef DIFFDRIVE_ARDUINO_BATTERY_H
#define DIFFDRIVE_ARDUINO_BATTERY_H

#include <string>



class Battery
{
    public:

    std::string name = "";

    double volt = 0;
    double curr = 0;
    

    Battery() = default;

    Battery(const std::string &battery_name);
    
    void setup(const std::string &battery_name);




};


#endif // DIFFDRIVE_ARDUINO_BATTERY_H