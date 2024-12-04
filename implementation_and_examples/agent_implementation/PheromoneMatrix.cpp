#include "PheromoneMatrix.h"
#include <algorithm>

PheromoneMatrix::PheromoneMatrix(double real_width, double real_height, double resolution) {
    this->x_min = 0- real_width / 2;
    this->x_max = real_width / 2;
    this->y_min = 0 - real_height / 2;
    this->y_max = real_height / 2;

    //Calculate the width and height of the matrix, and set the variables
    this->width = int(real_width / resolution) +1;
    this->height = int(real_height / resolution)+1;
    this->resolution = resolution;
    //Create the matrix and set all values to 0
    this->matrix = std::vector<std::vector<double>>(this->width, std::vector<double>(this->height, -1));
}

PheromoneMatrix::~PheromoneMatrix() {
    //Delete the matrix
    this->matrix.clear();
    std::vector<std::vector<double>>().swap(this->matrix);
}

void PheromoneMatrix::update(double x, double y, double visitedTimeS) {
    //Update the value of the matrix at the given coordinates
    //Convert to matrix coordinates
    int x_int = int((x - this->x_min)/ this->resolution);
    int y_int = int((y - this->y_min)/ this->resolution);

    //Take the maximum of the current value and the new value
    this->matrix[x_int][y_int] = std::max(this->matrix[x_int][y_int], visitedTimeS);
}

void PheromoneMatrix::update(Coordinate coordinate, double visitedTimeS) {
    //Update the value of the matrix at the given coordinates
    int x_int = int((coordinate.x - this->x_min) / this->resolution);
    int y_int = int((coordinate.y - this->y_min)/ this->resolution);

    //Take the maximum of the current value and the new value
    this->matrix[x_int][y_int] = std::max(this->matrix[x_int][y_int], visitedTimeS);
}

void PheromoneMatrix::updateByIndex(int x, int y, double visitedTimeS) {
    //Update the value of the matrix at the given coordinates
    this->matrix[x][y] = std::max(this->matrix[x][y], visitedTimeS);
}

void PheromoneMatrix::reset(Coordinate coordinate) {
    //Reset the value of the matrix at the given coordinates
    int x_int = int((coordinate.x - this->x_min) / this->resolution);
    int y_int = int((coordinate.y - this->y_min)/ this->resolution);
    this->matrix[x_int][y_int] = -1;
}

//void PheromoneMatrix::update(double x, double y, double value) {
//    //Update the value of the matrix at the given coordinates
//    //Convert to matrix coordinates
//    int x_int = int(x / this->resolution);
//    int y_int = int(y / this->resolution);
//    //Take the maximum of the current value and the new value
//    this->matrix[x_int][y_int] = std::max(this->matrix[x_int][y_int], value);
//}
//
//void PheromoneMatrix::update(Coordinate coordinate, double value) {
//    //Update the value of the matrix at the given coordinates
//    int x_int = int(coordinate.x / this->resolution);
//    int y_int = int(coordinate.y / this->resolution);
//    //Take the maximum of the current value and the new value
//    this->matrix[x_int][y_int] = std::max(this->matrix[x_int][y_int], value);
//}

double PheromoneMatrix::get(double x, double y, double currentTimeS) {
    //Get the value of the matrix at the given coordinates
    int x_int = int((x - this->x_min) / this->resolution);
    int y_int = int((y - this->y_min)/ this->resolution);
    double visitedTimeS = this->matrix[x_int][y_int];
    return calculatePheromone(visitedTimeS, currentTimeS);
}

double PheromoneMatrix::getByIndex(int x, int y, double currentTimeS) {
    //Get the value of the matrix at the given coordinates
    double visitedTimeS = this->matrix[x][y];
    return calculatePheromone(visitedTimeS, currentTimeS);
}

Coordinate PheromoneMatrix::getRealCoordinateFromIndex(int x, int y) {
    //Get the real world coordinates from the matrix coordinates
    double x_real = this->x_min + x * this->resolution;
    double y_real = this->y_min + y * this->resolution;
    return {x_real + this->resolution/2, y_real + this->resolution/2};
}

std::pair<int, int> PheromoneMatrix::getIndexFromRealCoordinate(Coordinate coordinate) {
    //Get the matrix coordinates from the real world coordinates

    int x_int = int((coordinate.x - this->x_min) / this->resolution);
    int y_int = int((coordinate.y - this->y_min)/ this->resolution);
    return {x_int, y_int};
}



double PheromoneMatrix::calculatePheromone(double visitedTimeS, double currentTimeS) const {
    if (visitedTimeS == -1) {
        return 0;
    }
    double pheromone = 1.0 -  std::min((currentTimeS - visitedTimeS) / EvaporationTime, 1.0);
    return pheromone;
}

//ONLY FOR COVERAGE MATRIX
//A cell is a frontier iff:
//1. Occupancy = explored, i.e. pheromone > 0
//2. At least one neighbor is unexplored (i.e. pheromone == 0) using the 8-connected Moore neighbours. (https://en.wikipedia.org/wiki/Moore_neighborhood)

//std::vector<std::pair<int, int>> PheromoneMatrix::getFrontierCells(double currentTimeS){
//    std::vector<std::pair<int, int>> frontierCells;
//    for (int i = 0; i < this->width; i++) {
//        for (int j = 0; j < this->height; j++) {
//            if(calculatePheromone(this->matrix[i][j], currentTimeS) > 0){ //Check if the cell is explored
//                //Check if at least one neighbor is unexplored
//                if (isMooreNeighbor0(i, j, currentTimeS)) {
//                    frontierCells.push_back({i, j});
//                }
//            }
//        }
//    }
//    return frontierCells;
//}

std::array<double, 9> PheromoneMatrix::MooreNeighbors(int x, int y, double currentTimeS){
    //Check if at least one of the 8-connected moore neighboring quadnodes of a given cell is unexplored.
    //If the neighbor is outside the matrix, it is considered unknown
    //If the neighbor is inside the matrix and has a pheromone value of 0, it is considered unknown
    //If the neighbor is inside the matrix and has a pheromone value > 0, it is considered explored

    std::array<double, 9> neighbors; //Index = (k+1) + 3*(l+1)

    for (int k = -1; k <= 1; k++) {
        for (int l = -1; l <= 1; l++) {
            //If neighbor (x+k, y+l) is inside the matrix
            if(x + k >= 0 && x + k < this->width && y + l >= 0 && y + l < this->height){
                //If the neighbor has a pheromone value of 0, it is considered unknown
                if(calculatePheromone(this->matrix[x + k][y + l], currentTimeS) == 0){
//                    return true;
                    neighbors[(k + 1) + 3*(l + 1)] = 0;
                } else {
                    neighbors[(k + 1) + 3*(l + 1)] = 1;
                }
            } else { //If the neighbor is outside the matrix, it is considered unknown
//                return true;
                neighbors[(k + 1) + 3*(l + 1)] = 0;
            }
        }
    }
//    return false;
    return neighbors;

}

std::string PheromoneMatrix::matrixToString(const std::string& rowDelimiter, const std::string& colDelimiter) {
    std::ostringstream oss;
    for (size_t i = 0; i < this->matrix.size(); ++i) {
        for (size_t j = 0; j < this->matrix[i].size(); ++j) {
            oss << this->matrix[i][j];
            if (j + 1 < this->matrix[i].size()) { // Add column delimiter if not the last element in the row
                oss << colDelimiter;
            }
        }
        if (i + 1 < this->matrix.size()) { // Add row delimiter if not the last row
            oss << rowDelimiter;
        }
    }
    return oss.str();
}