#ifndef PARSE_CONTROLLER_H
#define PARSE_CONTROLLER_H


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Script Information     //////////////////////////////////////////////////////////////////////////////

// Autor:       Patrick Schuurman
// Date:        15-11-2023
// Project:     Unilogger Software
// Filename:    parseController.h
// Changed:     03-01-2023 | 14:35
//
// Discription: This file contains the class to controll the parse arguments in terminal.


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Includes     ////////////////////////////////////////////////////////////////////////////////////////

//Header includes
#include "logCout.h"

// MISC Includes
#include <stdio.h>
#include <iostream>
#include <string>

// Using namespaces
using namespace std;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     Class: parse Controller      ///////////////////////////////////////////////////////////////////////

//  class to control parse arguments
class parseController
{
    private:

    public:
        // constructor
        parseController(void);

        // bools to control functions
        bool beginRecord;
        bool beginPlayback;
        bool getSerialNumber;
        bool getImage;
        bool getDepth;
        bool getSensordata;
        bool beginSvoToPcl;
        bool beginSvoToFilteredPcl;
        bool beginTrackPosition;
        bool beginSpatialMapping;
        bool beginTestWindow;
        bool beginPclFilter;
        bool beginPclViewer;
        bool beginPclFilterCompare;

        // functions
        bool argCountChecker(int &argc, char *argv1);
        bool argValueChecker(int &argc, char *argv1);
        bool argHelp(void) ;
        bool runFunction(char *argv2);

};


#endif // PARSE_CONTROLLER_H


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///     The End ;)     //////////////////////////////////////////////////////////////////////////////////////