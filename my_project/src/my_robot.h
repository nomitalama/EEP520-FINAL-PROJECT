#ifndef __MY_ROBOT_AGENT__H
#define __MY_ROBOT_AGENT__H 

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <iomanip>
#include "enviro.h"
#include "random_maze.h"



namespace {
    using namespace enviro;
    
    const double BL = 20.0; //Body length is length of robot.
    const double UP = -0.5*M_PI; //pi/2 or 90deg.
    const double DOWN = -1.5*M_PI; //pi*3/4 or 270deg.
    const double LEFT = M_PI; //pi or 180deg.
    const double RIGHT = 0; //0 or 0deg.
    const double speed = 90; // how fast Binky moves.

    void makeJsonFile(double cellDim) {
        RandomMaze randomMaze(15, 15); //size of the maze (width(number > 6), heigth(every othe odd > 6)) 
        std::vector<json> statics = randomMaze.getCellData(cellDim);
        json j;
        json agent;
        json style;
        json position;
        json statik; //Temporary json to use during for loop below...
        json shape1; //Temporary json to use during for loop below...
        json shape2;
        json shape3;
        json shape4;

        style["fill"] = "green"; //Binky's color.
        style["stroke"] = "none";

        position["x"] = 20;
        position["y"] = 210;
        position["theta"] = 0;

        agent["definition"] = "defs/my_robot.json";
        agent["style"] = style;
        agent["position"] = position;

        vector<json> agents; //AGENTS
        agents.push_back(agent);

        vector<json> shapes; //SHAPES
        
        j["statics"] = statics;
        j["agents"] = agents;
        j["port"] = 8765;
        j["ip"] = "0.0.0.0";
        j["name"] = "My Enviro Project"; //Takes the name of the statemachine and stores it as a json object called "name".

        std::ofstream outfile ("config.json");
        outfile << j;
        outfile.close();
    }
    //Binky repositions himself to stay one body length away from the right wall.
    class Repositioning : public State, public AgentInterface {  
        public:
        void entry(const Event& e) {
            std::cout << std::setprecision(10);
            initR = sensor_value(1);
            initX = this->position().x;
            initY = this->position().y;
            initA = this->angle();
        }
        void during() {
            double diff = initR - BL; //this is how Binky calculates where to reposition himself by teleporting
            if(initA == UP || initA == -DOWN) {
                std::cout << "Repositioning UP: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << initA << " " << "\n"; //Prints direction robot's facing, NOT direction he has to move to reposition.
                teleport(initX + diff, initY, initA);
                emit(Event(repositioned));
            }
            else if(initA == DOWN || initA == -UP) {
                std::cout << "Repositioning DOWN: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << initA << " " << "\n"; //Prints direction robot's facing, NOT direction he has to move to reposition.
                teleport(initX - diff, initY, initA);
                emit(Event(repositioned));
            }
            else if(initA == LEFT || initA == -LEFT) {
                std::cout << "Repositioning LEFT: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << initA << " " << "\n"; //Prints direction robot's facing, NOT direction he has to move to reposition.
                teleport(initX, initY - diff, initA);
                emit(Event(repositioned));
            }
            else if(initA == RIGHT) {
                std::cout << "Repositioning RIGHT: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << initA << " " << "\n"; //Prints direction robot's facing, NOT direction he has to move to reposition.
                teleport(initX, initY + diff, initA);
                emit(Event(repositioned));
            }
            else {
                std::cout << "Repositioning ERROR\n";
                while(1) {}
            }
        }
        void exit(const Event& e) {}
        void set_tick_name(std::string s) { repositioned = s; }
        double changeAngle(double a, bool cw) { return (cw) ? a - UP : a + UP; }
        std::string repositioned;

        double initR = 0; //Initial value of RHS sensor (sensor_value(1)).
        double initX = 0; //Initial x position.
        double initY = 0; //Initial y position.
        double initA = 0; //Initial angle in radians.
        double newA = 0; //New angle in radians.
    };
    //Binky moves in a straight path, one body length away from the right wall toward an assumed cocave corner, unless he detects a convex corner. 
    class DynamicMoving : public State, public AgentInterface { 
        public: 
        void entry(const Event& e) {
            initN = sensor_value(0);
            initR = sensor_value(1);
            initX = this->position().x;
            initY = this->position().y;
            initA = this->angle();
        }
        void during() { 
            if(initA == UP || initA == -DOWN) {
                std::cout << "Dynamic moving UP: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << this->position().y - (initY - initN + BL) << "\n";
                if(sensor_value(1) > initR) //Binky detects convex corner.
                    emit(Event(handleConvex));
                else if(this->position().y - (initY - initN + BL) < 10E-12)
                    emit(Event(handleConcave));
                std::cout << "Dynamic moving UP: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << this->position().y - (initY - initN + BL) << "\n";
                move_toward(initX, initY - initN + BL, speed, 0);
            }
            else if(initA == DOWN || initA == -UP) {
                std::cout << "Dynamic moving DOWN: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << (initY + initN - BL) - this->position().y << "\n";
                if(sensor_value(1) > initR) //Binky detects convex corner.
                    emit(Event(handleConvex));
                else if((initY + initN - BL) - this->position().y < 10E-12) 
                    emit(Event(handleConcave));
                std::cout << "Dynamic moving8 DOWN: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << (initY + initN - BL) - this->position().y << "\n";
                move_toward(initX, initY + initN - BL, speed, 0);
            }
            else if(initA == LEFT || initA == -LEFT) {
                std::cout << "Dynamic moving LEFT: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << this->position().x - (initX - initN + BL) << "\n";
                if(sensor_value(1) > initR) //Binky detects convex corner.
                    emit(Event(handleConvex));
                else if(this->position().x - (initX - initN + BL) < 10E-12)
                    emit(Event(handleConcave));
                std::cout << "Dynamic moving LEFT: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << this->position().x - (initX - initN + BL) << "\n";
                move_toward(initX - initN + BL, initY, speed, 0);
            }
            else if(initA == RIGHT) {
                std::cout << "Dynamic moving RIGHT: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << (initX + initN - BL) - this->position().x << "\n";
                if(sensor_value(1) > initR) //Binky detects convex corner.
                    emit(Event(handleConvex));
                else if((initX + initN - BL) - this->position().x < 10E-12)
                    emit(Event(handleConcave))
                std::cout << "Dynamic moving RIGHT: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << (initX + initN - BL) - this->position().x << "\n";
                move_toward(initX + initN - BL, initY, speed, 0);
            }
            else {
                std::cout << "Dynamic moving ERROR" << "\n";
                while(1){}
            }
        }
        void exit(const Event& e) { std::cout << "Dynamic moving EXIT: " << sensor_value(0) << " " << sensor_value(1) << " " << "" << initY << " " << initA << "\n"; }
        void set_tick_name(std::string s1, std::string s2) { handleConcave = s1; handleConvex = s2; }
        double changeAngle(double a, bool cw) { return (cw) ? a - UP : a + UP; }
        std::string handleConcave;
        std::string handleConvex;

        double initN = 0; //Initial value of sensor_value(0).
        double initR = 0; //Initial value of sensor_value(1).
        double initX = 0; //Initial x position.
        double initY = 0; //Initial y position.
        double initA = 0; //Initial angle in radians.
    };
    //After Binky detects a convvex corner, he stops very briefly. 
    class Stopping : public State, public AgentInterface {
        public:
        void entry(const Event& e) {
            initX = this->position().x;
            initY = this->position().y;
            initA = this->angle();
        }
        void during() {
            if(this->velocity().x == 0 && this->velocity().y == 0)
                emit(Event(stopped));
            else 
                teleport(initX, initY, initA);
        }
        void exit(const Event& e) {}
        void set_tick_name(std::string s) { stopped = s; }
        std::string stopped;

        double initX = 0; //Initial x position.
        double initY = 0; //Initial y position.
        double initA = 0; //Initial angle in radians.
    };
    //Binky moves forward one body length. 
    class StaticMoving : public State, public AgentInterface {
        public:
        void entry(const Event& e) {
            initX = this->position().x;
            initY = this->position().y;
            initA = this->angle();
        }
        void during() {
            if(initA == UP || initA == -DOWN) {
                std::cout << "Static moving UP: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << this->position().y - (initY - BL) << "\n";
                
                if(this->position().y - (initY - BL) < 10E-12)
                    emit(Event(staticMoved));
                move_toward(initX, initY - BL, speed, 0);
            }
            else if(initA == DOWN || initA == -UP) {
                std::cout << "Static moving DOWN: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << (initY + BL) - this->position().y<< "\n";
                
                if((initY + BL) - this->position().y < 10E-12)
                    emit(Event(staticMoved));
                move_toward(initX, initY + BL, speed, 0);
            }
            else if(initA == LEFT || initA == -LEFT) {
                std::cout << "Static moving LEFT: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << this->position().x - (initX - BL) << "\n";
                
                if(this->position().x - (initX - BL) < 10E-12)
                    emit(Event(staticMoved));
                move_toward(initX - BL, initY, speed, 0);
            }
            else if(initA == RIGHT) {
                std::cout << "Static moving RIGHT: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << (initX + BL) - this->position().x << "\n";
                
                if((initX + BL) - this->position().x < 10E-12)
                    emit(Event(staticMoved));
                move_toward(initX + BL, initY, speed, 0);
            }
            else {
                std::cout << "Static moving ERROR: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << "\n";
                while(1){}
            }
        }
        void exit(const Event& e) {}
        void set_tick_name(std::string s) { staticMoved = s; }
        double changeAngle(double a, bool cw) { return (cw) ? a - UP : a + UP; }
        std::string staticMoved;
        double initX = 0; //Initial x position.
        double initY = 0; //Initial y position.
        double initA = 0; //Initial angle in radians.
    };
    //Binky rotates clockwise by 90 degrees
    class ConvexRotating : public State, public AgentInterface {
        public:
        void entry(const Event& e) { 
            i = 5;
            initX = this->position().x;
            initY = this->position().y;
            initA = this->angle();
            targetA = changeAngle(initA, true);
        }
        void during() {
            if(i == 0)
                emit(Event(convexRotated));
            else {
                if(initA == UP || initA == -DOWN) {
                    std::cout << "Convex rotating UP: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << "\n";
                    teleport(initX, initY, initA - UP/i);
                }
                else if(initA == DOWN || initA == -UP) {
                    std::cout << "Convex rotating DOWN: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << "\n";
                    teleport(initX, initY, (initA - UP/i));
                }
                else if(initA == LEFT || initA == -LEFT) {
                    std::cout << "Convex rotating LEFT: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << "\n";
                    teleport(initX, initY, (initA - UP/i));
                }
                else if(initA == RIGHT) {
                    std::cout << "Convex rotating RIGHT: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << "\n";
                    teleport(initX, initY, (initA - UP/i));
                }
                else {
                    std::cout << "Convex rotating ERROR: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << "\n";
                    while(1){}
                }
                i--;
            }
        }
        void exit(const Event& e) {
            if(this->angle() == 2*LEFT)
                teleport(initX, initY, 0);
            std::cout << "ConvexRotating EXIT: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << "\n";
        }
        void set_tick_name(std::string s) { convexRotated = s; }
        double changeAngle(double a, bool cw) { return (cw) ? a - UP : a + UP; }
        std::string convexRotated;
        int i = 0; // this is itteration variable
        double targetA = 0; // target angle for rotation
        double initX = 0; //Initial x position.
        double initY = 0; //Initial y position.
        double initA = 0; //Initial angle in radians.        
    };
    //Binky rotates counterclockwise by 90 degrees.
    class ConcaveRotating : public State, public AgentInterface {
        public:
        void entry(const Event& e) { 
            i = 5;
            initX = this->position().x;
            initY = this->position().y;
            initA = this->angle();
            targetA = changeAngle(initA, true);
        }
        void during() {
            if(i == 0)
                emit(Event(concaveRotated));
            else {
                if(initA == UP || initA == -DOWN) {
                    std::cout << "Concave rotating UP: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << "\n";
                    teleport(initX, initY, initA + UP/i);
                }
                else if(initA == DOWN || initA == -UP) {
                    std::cout << "Concave rotating DOWN: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << "\n";
                    teleport(initX, initY, (initA + UP/i));
                }
                else if(initA == LEFT || initA == -LEFT) {
                    std::cout << "Concave rotating LEFT: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << "\n";
                    teleport(initX, initY, (initA + UP/i));
                }
                else if(initA == RIGHT) {
                    std::cout << "Concave rotating RIGHT: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << " " << "\n";
                    teleport(initX, initY, (initA + UP/i));
                }
                else {
                    std::cout << "Concave rotating ERROR: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << "\n";
                    while(1){}
                }
                i--;
            }
        }
        void exit(const Event& e) {
            if(abs(this->angle()) == 2*LEFT)
                teleport(initX, initY, 0);
            std::cout << "Concave rotating EXIT: " << sensor_value(0) << " " << sensor_value(1) << " " << this->angle() << "\n";
        }
        void set_tick_name(std::string s) { concaveRotated = s; }
        double changeAngle(double a, bool cw) { return (cw) ? a - UP : a + UP; }
        void test() {
        }
        std::string concaveRotated;
        int i = 0; // this is itteration variable
        double targetA = 0; // target angle for rotation
        double initX = 0; //Initial x position.
        double initY = 0; //Initial y position.
        double initA = 0; //Initial angle in radians.        
    };
    //Binky's central nervous system. He is a simple creature who moves form state to state when triggered by a stimuli. 
    class MyRobotController : public StateMachine, public AgentInterface {
        public:
        MyRobotController() : StateMachine() {
            makeJsonFile(60);
            
            set_initial(repositioning);
            string temp = std::to_string(rand()%1000);
            convexRotated = "convexRotated_" + temp; //Use an agent specific generated event name in case there are multiple instances of this class.
            concaveRotated = "concaveRotated_" + temp;
            repositioned = "repositioned_" + temp;
            handleConcave = "handleConcave_" + temp;
            handleConvex = "handleConvex_" + temp;
            staticMoved = "staticMoved_" + temp;
            staticMoved2 = "staticMoved2_" + temp;
            staticMoved3 = "staticMoved3_" + temp;
            stopped = "stopped_" + temp;
            tested = "tested_" + temp;
            add_transition(repositioned, repositioning, dynamicMoving);
            add_transition(handleConcave, dynamicMoving, concaveRotating);
            add_transition(handleConvex, dynamicMoving, stopping);
            add_transition(stopped, stopping, staticMoving);
            add_transition(staticMoved, staticMoving, convexRotating); //1
            add_transition(convexRotated, convexRotating, staticMoving2);
            add_transition(concaveRotated, concaveRotating, repositioning);
            add_transition(staticMoved2, staticMoving2, staticMoving3); //2
            add_transition(staticMoved3, staticMoving3, repositioning); //3
            repositioning.set_tick_name(repositioned);
            staticMoving.set_tick_name(staticMoved);
            staticMoving2.set_tick_name(staticMoved2);
            staticMoving3.set_tick_name(staticMoved3);
            stopping.set_tick_name(stopped);
            convexRotating.set_tick_name(convexRotated);
            concaveRotating.set_tick_name(concaveRotated);
            dynamicMoving.set_tick_name(handleConcave, handleConvex);
            testing.set_tick_name(tested);
        }

        StaticMoving staticMoving; //Using
        StaticMoving staticMoving2;
        StaticMoving staticMoving3;
        Stopping stopping; //Using
        ConvexRotating convexRotating;
        ConcaveRotating concaveRotating;
        Repositioning repositioning; //Using
        DynamicMoving dynamicMoving; //Using
        Testing testing;
        std::string convexRotated;
        std::string concaveRotated;
        std::string staticMoved;
        std::string staticMoved2;
        std::string staticMoved3;
        std::string stopped;
        std::string repositioned;
        std::string handleConcave;
        std::string handleConvex;
        std::string tested;
    };

    class MyRobot : public Agent {
        public:
        MyRobot(json spec, World& world) : Agent(spec, world) {
            add_process(c);
        }
        private:
        
        MyRobotController c;
    };

    DECLARE_INTERFACE(MyRobot)

}
#endif