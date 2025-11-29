#include <iostream>
#include <vector>
#include <string>
#include <ctime>
#ifndef ROBOTOBJECTHPP
#define ROBOTOBJECTHPP
class Robot
{
    public:
        char robotName;
        std::string robotType;
        
   Robot(char robotName, std::string robotType)
    {
        this->robotName = robotName;
        this->robotType = robotType;
    }

    char getName()
    {
        return robotName;
    }

    std::string getType()
    {
        return robotType;
    }
 };

#endif 
