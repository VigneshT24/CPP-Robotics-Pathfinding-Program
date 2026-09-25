#include <iostream>
#include <vector>
#include <string>
#include <ctime>
#ifndef ROBOTOBJECTHPP
#define ROBOTOBJECTHPP
class Robot {
    private:
        char robotName;
        std::string robotType;
    
    public:
        Robot(char robotName, std::string robotType) {
            this->robotName = robotName;
            this->robotType = robotType;
        }

        char getName() const {
            return robotName;
        }

        std::string getType() const {
            return robotType;
        }
 };

#endif 
