#ifndef SERIALCOMM_H
#define SERIALCOMM_H

#include <boost/asio.hpp> // include boost
#include <boost/asio/serial_port.hpp>
#include <iostream>
#include <string>

using namespace std;
using namespace boost::asio;

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

 // -------- Reading methods declarations. Implementation in SerialComm.cpp --------

    bool ReadChar(); //Read a single char
    char GetChar(); //Get the read char

    bool ReadNumberofChars(int); //Read a specified number of charts specified by the user
    string GetNumberofChars(int); //Read a specified number of charts specified by the user and get it

    bool ReadLine(); //Read a line till final carriage \n
    string GetLine(); //Read a line till final carriage \n and get it

    bool ReadUntill (char); //Read until an ending condition specified by the user

 // -------- Writing methods declarations. Implementation in SerialComm.cpp --------

    bool WriteLine(string); //Write a line via serial comm

 // -------- Checking method declaration. Implementation in SerialComm.cpp --------

    bool CheckLine(string); //Read data and compare it with a given string by the user

private: //Attributes

    io_context io; //Active I/0 Functions
    serial_port *port; //Creation of the object
    boost::system::error_code error;
    boost::asio::streambuf buffer;

};

#endif // SERIALCOMM_H
