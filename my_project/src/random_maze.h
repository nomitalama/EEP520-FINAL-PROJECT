#include <bits/stdc++.h>
#include <vector>
#include <tuple>
#include <iostream> 
#include <json/json.h>
using nlohmann::json;
//generates a random maze for Binky to solve
class RandomMaze {
    public:
        RandomMaze() { //Default constructor 
            srand(time(NULL));
            _width = 7;
            _height = 7;
            initialize();
            entryAndExit(true);
            entryAndExit(false);
            generate();
        }
        RandomMaze(int width, int height) { //Parametrized Constructor
            srand(time(NULL));
            _width = width;
            _height = height;
            initialize();
            entryAndExit(true);
            entryAndExit(false);
            generate();
        }
        std::vector<json> getCellData(double cellDim); //Returns list of squares each w/ specified dimension.
    
    private:
        std::vector<std::vector<int>> _dfsPath;
        std::vector<std::vector<std::vector<bool>>> _maze;
        int _width; //Width (Positive odd int i > 6).
        int _height; //Height (Positive odd int i > 6 s.t. 0.5*(i + 1)%2 == 0).
        void initialize();
        void entryAndExit(bool entry);
        void generate();
        bool move(bool initMove); //Append random direction to current path & move there.
};

std::vector<json> RandomMaze::getCellData(double cellDim) {
    std::vector<json> statics; //STATICS
    std::vector<json> shapes; //SHAPES
    json statik; //Temporary json to use during for loop below...
    json shape1; //Temporary json to use during for loop below...
    json shape2;
    json shape3;
    json shape4;
    json style;
    style["fill"] = "black"; //Color of statics
    style["stroke"] = "none";

    int i;
    int columns = _maze[0].size();
    int rows = _maze.size();
    
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < columns; c++) {
            if (_maze[r][c][0]) {
                i = r*columns + c;
                statik["style"] = style;
                shape1["x"] = cellDim*c;
                shape1["y"] = cellDim*r;
                shape2["x"] = cellDim*(c + 1);
                shape2["y"] = cellDim*r;
                shape3["x"] = cellDim*(c + 1);
                shape3["y"] = cellDim*(r + 1);
                shape4["x"] = cellDim*c;
                shape4["y"] = cellDim*(r + 1);
                shapes.push_back(shape1);
                shapes.push_back(shape2);
                shapes.push_back(shape3);
                shapes.push_back(shape4);

                statik["shape"] = shapes;
                statics.push_back(statik);
            }
            shapes.clear();
        }
    }
    
    return statics;
}

//Makes a maze of the specified dimensions and initializes every cell to true, i.e., true = wall; false = no wall.
void RandomMaze::initialize() {
    for (int a = 0; a < _height; a++) {
        for (int b = 0; b < _width; b++) {
            bool isBorder = (a == 0 || a == _height - 1 || b == 0 || b == _width - 1);
            std::vector<bool> cell = {true, isBorder};
            if ((unsigned int)a + 1 > _maze.size())
                _maze.push_back({cell});
            else
                _maze[a].push_back(cell);
        }
    }
}

//This is where the entry and exit location are set. If true is passed, then entry is set, o/w exit is set.
void RandomMaze::entryAndExit(bool entry) {
    std::vector<int> location = {0, 0};
    if(entry) { //Set the entry location.
        location[0] = 0; //X component of entry location.
        location[1] = 3; //Y component of entry location.
        _dfsPath.push_back(location); //I.e., entry location is always the same.
    }
    else { //Set the exit location.
        int exitSide; //The side that the exit will be located on.
        int exitAxis; //The axis that the exit will be located on.
        while(1) {
            exitAxis = rand()%2; //Randomly assign an axis.
            exitSide = rand()%2; //Randomly assign a side.
            if (exitAxis != 1 || exitSide != 0) //Don't put the exit on the same side as the entry.
                break;
        }
        location[!exitAxis] = (exitAxis) ? exitSide*(_width - 1) : exitSide*(_height - 1);
        location[exitAxis] = (exitAxis) ? 2*(rand()%((_width + 1)/2 - 2)) + 1 : 2*(rand()%((_height + 1)/2 - 2)) + 1; //Exit location.
    }
    _maze[location[1]][location[0]][0] = false;
    _maze[location[1]][location[0]][1] = true;
}

//Generates the maze by deleting "walls" @ certain cells.
void RandomMaze::generate() {
    bool initMove = true; //Set to true so we know if we are on the first move.
    bool success = true;
    while ((int)_dfsPath.size() > 1 - initMove) {
        if (!success) {
            _dfsPath.pop_back();
            if (!initMove && _dfsPath.size() > 2)
                _dfsPath.pop_back();
            else
                break;
            success = true;
        }
        while (success) {
            success = move(initMove);
            initMove = false;
        }
    }
}

//Move a distance in the maze.
bool RandomMaze::move(bool initMove) { 
    int randomNeighbor;
    std::vector<std::vector<int>> unvisitedNeighbors;
    for (int dir = 0; dir < 4; dir++) {
        int possiblePmd[2] = {0, 0};
        possiblePmd[(dir < 2)] = (dir < 2) ? -1 + 2*(dir == 1) : -1 + 2*(dir == 3);
        if (_dfsPath.back()[0] + possiblePmd[0]*2 > 0 && _dfsPath.back()[0] + possiblePmd[0]*2 < _width - 1 && _dfsPath.back()[1] + possiblePmd[1]*2 > 0 && _dfsPath.back()[1] + possiblePmd[1]*2 < _height - 1) {
            if (!_maze[_dfsPath.back()[1] + possiblePmd[1]*2][_dfsPath.back()[0] + possiblePmd[0]*2][1])
                unvisitedNeighbors.push_back({possiblePmd[0], possiblePmd[1]});
        }
    }
    if (unvisitedNeighbors.size() > 0) {
        randomNeighbor = rand()%unvisitedNeighbors.size();
        for (int a = 0; a < !initMove + 1; a++) {
            std::vector<int> newLocation;
            newLocation.push_back(_dfsPath.back()[0] + unvisitedNeighbors[randomNeighbor][0]);
            newLocation.push_back(_dfsPath.back()[1] + unvisitedNeighbors[randomNeighbor][1]);
            _dfsPath.push_back(newLocation);
            _maze[_dfsPath.back()[1]][_dfsPath.back()[0]][0] = false;
            _maze[_dfsPath.back()[1]][_dfsPath.back()[0]][1] = true;
        }
        return true;
    }
    return false;
}
