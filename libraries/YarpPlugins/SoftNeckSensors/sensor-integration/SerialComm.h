#ifndef SERIALCOMM_H
#define SERIALCOMM_H

#include <boost/asio.hpp> // include boost
#include <boost/asio/serial_port.hpp>
#include <iostream>
#include <string>

using namespace std;
using namespace boost::asio;
//using namespace boost;

/*
// These are the values our port needs to connect
#ifdef _WIN64
// windows uses com ports, this depends on what com port your cable is plugged in to.
const char *PORTNAME = "COM3";
#else
// *nix com ports
    const char *PORTNAME = "/dev/ttyUSB0";
#endif
*/

class SerialComm
{
public:
    SerialComm(string portName = "/dev/ttyUSB0"); //Constructor

    //Read functions
    string ReadLine(); //Read a line till final carriage \n
    char ReadChar(); //Read a single char
    string ReadNumberofChars(int); //Read a set of charts specified by the user
    string ReadUntill (char); //Read until an ending condition specified by the user
    int CheckLine(string); //Read data and compare it with a given string by the user

    //Write functions
    long WriteLine(string); //Write a line via serial comm

private: //Attributes

    io_context io; //Active I/0 Functions
    serial_port *port; //Creation of the object
    boost::system::error_code error;
    boost::asio::streambuf buffer;



};

#endif // SERIALCOMM_H
