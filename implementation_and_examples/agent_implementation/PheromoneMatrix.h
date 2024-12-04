#ifndef IMPLEMENTATION_AND_EXAMPLES_PHEROMONEMATRIX_H
#define IMPLEMENTATION_AND_EXAMPLES_PHEROMONEMATRIX_H

#include "coordinate.h"
#include <array>

class PheromoneMatrix {
public:
    PheromoneMatrix() = default;
    PheromoneMatrix(double real_width, double real_height, double resolution);
    ~PheromoneMatrix();
    void update(double x, double y, double visitedTimeS);
    void update(Coordinate coordinate, double visitedTimeS);
    void reset(Coordinate coordinate);
    double get(double x, double y, double currentTimeS);
    double getByIndex(int x, int y, double currentTimeS);
    Coordinate getRealCoordinateFromIndex(int x, int y);
    std::pair<int,int> getIndexFromRealCoordinate(Coordinate coordinate);
    std::vector<std::pair<int, int>> getFrontierCells(double currentTimeS);

    std::vector<std::vector<double>> getMatrix() const { return this->matrix; }

    void evaporate(double rate);
    void print();
    double getResolution() const { return this->resolution; }
    double getRealWidth() const { return this->width * this->resolution; }
    double getRealHeight() const { return this->height * this->resolution; }
    double getWidth() const { return this->width; }
    double getHeight() const { return this->height; }
    double calculatePheromone(double visitedTime, double currentTime) const;
    std::array<double, 9> MooreNeighbors(int x, int y, double currentTimeS);

private:
    std::vector<std::vector<double>> matrix;

    //Meters
    double x_min;
    double x_max;
    double y_min;
    double y_max;

    //N-cells
    int width;
    int height;

    double resolution;
    static constexpr double EvaporationTime = 100.0;

//    void update(double x, double y, double value);
//    void update(Coordinate coordinate, double value);


    };


#endif //IMPLEMENTATION_AND_EXAMPLES_PHEROMONEMATRIX_H
