/** Author: Anna Font Llenas and Andreagiovanni Reina a.reina@sheffield.ac.uk
 *  Copyright University of Sheffield, 2018
 *  If you use this code for scientific experiment, please cite:
 *  A. Font Llenas et al. 2018 in ANTS 2018
 */

#ifndef MYKILOBOTENVIRONMENT_H
#define MYKILOBOTENVIRONMENT_H

#include <QObject>
#include <QPointF>
#include <QVector>
#include <QVector3D>
#include <kilobotenvironment.h>
#include <QTime>
#include <QMatrix>
#include <QList>
#include <QColor>
#include <QElapsedTimer>
#include <limits>

struct foodClass {
    float posX;
    float posY;
    int quality;
    int rad;
};

struct infoClass {
    kilobot_id id;
    int source;
    int time;
};

//struct positionClass {
//    kilobot_id id;
//    float time;
//    float posX;
//    float posY;
//};

//list.append(QVariant(tmp));

//...

//MyStruct tmp2 = list.at(0).value<foodClass>();



class mykilobotenvironment : public KilobotEnvironment
{
    Q_OBJECT
public:
    explicit mykilobotenvironment(QObject *parent = 0);
    void reset();

    QList<foodClass> foodList;
    QList<infoClass> infoList;
    //QList<int **> pheroList;
    float HOME_X;
    float HOME_Y;
    int NumFood;
    float evaporation_rate;
    float diffusion_rate;
    int pheromone_amount;
    int radHome = 100;
    int ArenaX, ArenaY;
    int MatrixSize_x;
    int MatrixSize_y;
    float minTimeBetweenTwoMsg;
    double time;
    bool ongoingRuntimeIdentification;
    QVector < float >  lastSent;
    QVector < int >  hasFood; // 0 is no food, and food>0 represents the food source
    QVector < bool >  atPheromone; // at pheromone for logging
    QVector < bool >  isPrinting;

//    kilobot_colour kCol;
//    QPointF kPos;
    QPointF HomePos;
    QPointF orientDegrees;

    // phero matrix params
    float last_matrix_update;
    float matrix_update_time = 0.5;
    float *floorMatrix;
    float *aux_floorMatrix;
    float maxPheromoneValue = std::numeric_limits<float>::max()-1000;

signals:
    void errorMessage(QString);

public slots:
    void update();
    void updateVirtualSensor(Kilobot kilobot);
    uint16_t pheroZonesCalculation(float track_x, float track_y, double orientDegrees);
    float normAngle(float angle);
    float desNormAngle(float angle);
private:

};




#endif // MYKILOBOTENVIRONMENT_H
