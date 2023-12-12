/********************************************************
 * Controller-in-Cloud: Recorder
 * 
 * Description:
 * Recorder class to store data for plotting in execution
 * 
 * Author: 
 * Hamid Sadeghian
 * Munich Institure of Robotics and System Intelligence
 * Technical University of Munich
 * hamid.sadeghian@tum.de
 * 
*********************************************************/

#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <iostream>

using namespace std;
using namespace Eigen;

class Recorder {
   public:
    Recorder(double t_rec, double sampleTime, int NoDataRec = 10, std::string name = "DATA", std::string path = "./");
    ~Recorder();

    void addToRec(int value);
    void addToRec(double value);
    void addToRec(double array[], int sizeofarray);
    void addToRec(std::array<double, 3> array);
    void addToRec(std::array<double, 6> array);
    void addToRec(std::array<double, 7> array);

    void addToRec(VectorXd vector);
    void addToRec(double* array);
    void saveData();
    void next();

    void getDAT(Matrix<double, Dynamic, 1>& _Buffer, int rowNum);

   private:
    int _index;
    int _columnindex;
    int _rowindex;
    double _t_rec;
    int _NoDataRec;
    std::string _name;
    std::string _path;
    Matrix<double, Dynamic, Dynamic> _DAT;
};