#include "mbed.h"
//Setup a new class for TMP36 sensor
class TMP36
{
public:
    TMP36(PinName pin);
    TMP36();
    operator float ();
    float read();
private:
//class sets up the AnalogIn pin
    AnalogIn _pin;
};
 
TMP36::TMP36(PinName pin) : _pin(pin)
{
// _pin(pin) means pass pin to the AnalogIn constructor
}
 
float TMP36::read()
{
//convert sensor reading to temperature in degrees C
    return ((_pin.read()*3.3)-0.500)*10000;
}
//overload of float conversion (avoids needing to type .read() in equations)
TMP36::operator float ()
{
//convert sensor reading to temperature in degrees C
    return ((_pin.read()*3.3)-0.500)*10000;
}
TMP36 myTMP36(A0);