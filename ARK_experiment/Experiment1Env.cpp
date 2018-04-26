/** Author: Anna Font Llenas and Andreagiovanni Reina a.reina@sheffield.ac.uk
 *  Copyright University of Sheffield, 2018
 *  If you use this code for scientific experiment, please cite:
 *  A. Font Llenas et al. 2018 in ANTS 2018
 */

#include "Experiment1Env.h"
#include <QVector>
#include <QLineF>
#include <QDebug>
#include <QtMath>
#include <QColor>
#include "kilobot.h"

#define I2I(x,y) int(x)*MatrixSize_x+int(y)

mykilobotenvironment::mykilobotenvironment(QObject *parent) : KilobotEnvironment(parent) {

    ArenaX = 2;
    ArenaY = 2;
    int cells_per_metre = 150;
    MatrixSize_x = ArenaX*cells_per_metre;
    MatrixSize_y = ArenaY*cells_per_metre;

    // Define Environment:
    // call any functions to setup features in the environment (goals, homes locations and parameters).
    reset();

    NumFood = 0;
    evaporation_rate = 0.08;
    diffusion_rate = 0.01;
    HOME_X = 1.0;
    HOME_Y = 1.0;
    pheromone_amount = 100;
    ongoingRuntimeIdentification = false;

}

void mykilobotenvironment::reset(){
    time = 0;
    minTimeBetweenTwoMsg = 0;
    last_matrix_update = 0;
    ongoingRuntimeIdentification = false;

    //Create the FloorMatrix
    floorMatrix = (float*) malloc(MatrixSize_x*MatrixSize_y*sizeof(float));
    aux_floorMatrix = (float*) malloc(MatrixSize_x*MatrixSize_y*sizeof(float));
    memset(floorMatrix, 0, MatrixSize_x*MatrixSize_y*sizeof(float));
//    // Initialize the FloorMatrix
//    for(int i = 0; i < MatrixSize_x; i++){
//        for(int j = 0; j < MatrixSize_y; j++){
//            floorMatrix[i][j] = 0;
//        }
//    }

}

// Only update if environment is dynamic:
void mykilobotenvironment::update() {
    //Save the previous matrix in an static array
    memcpy(aux_floorMatrix, floorMatrix, MatrixSize_x*MatrixSize_y*sizeof(float));
    float diff_I, diff_J;
    float diffusionComponent = 0;

    //All except the borders
    for(int i = 1; i < MatrixSize_x-1; i++){
        for(int j = 1; j < MatrixSize_y-1; j++){
            //Calculate the vertical diffusion
            diff_I = aux_floorMatrix[I2I(i+1,j)]-2*aux_floorMatrix[I2I(i,j)]+aux_floorMatrix[I2I(i-1,j)];
            //Calculate the horizontal diffusion
            diff_J = aux_floorMatrix[I2I(i,j+1)]-2*aux_floorMatrix[I2I(i,j)]+aux_floorMatrix[I2I(i,j-1)];

            //Evaporation and diffusion equation
            diffusionComponent = diffusion_rate*(diff_I + diff_J)*matrix_update_time;
            // if (diffusionComponent > maxPheromoneValue) diffusionComponent = maxPheromoneValue;
            floorMatrix[I2I(i,j)] = diffusionComponent - (evaporation_rate*matrix_update_time - 1)*aux_floorMatrix[I2I(i,j)];
            floorMatrix[I2I(i,j)] = (floorMatrix[I2I(i,j)] > maxPheromoneValue)? maxPheromoneValue : floorMatrix[I2I(i,j)];
            floorMatrix[I2I(i,j)] = (floorMatrix[I2I(i,j)] < 0.5)? 0.0 : floorMatrix[I2I(i,j)];

        }
    }
    last_matrix_update = time;
}

float mykilobotenvironment::normAngle(float angle){
    while (angle > 180) angle = angle - 360;
    return angle;
}

float mykilobotenvironment::desNormAngle(float angle){
    while (angle > 360) angle = angle - 360;
    while (angle < 0) angle = 360 + angle;
    return angle;
}

//Calculate the orientation of the kilobots and save the floor values that the antenae can feel
uint16_t mykilobotenvironment::pheroZonesCalculation(float track_x, float track_y, double orientDegrees){

    uint16_t pheroZones = 0;
    int radDetection = 5;
    float angleToKilobot;
    QPointF track_xy;
    QPointF newPos;
    int phToSend[4] = {0,0,0,0};
    //If it is in the corners, don't substract the numbers
//    if(track_x > 295) track_x = 295;
//    if(track_y > 295) track_y = 295;
//    if(track_x < 4) track_x = 4;
//    if(track_y < 4) track_y= 4;

    //Check if pheromone in their surrondings
    for(int i = qMax(0,(int)(track_x - radDetection)); i < qMin(MatrixSize_x-1, (int)(track_x + radDetection)); i++){
        for(int j = qMax(0,(int)(track_y - radDetection)); j < qMin(MatrixSize_y-1, (int)(track_y + radDetection)); j++){
            if(floorMatrix[I2I(i,j)] > 0){
                //Calculate the path to ph
                track_xy.setX(track_x);
                track_xy.setY(track_y);
                newPos.setX(i);
                newPos.setY(j);
                QLineF path(track_xy,newPos);

                //Add orientation
//                if(orientDegrees < 0){
//                    orientDegrees =  orientDegrees + 360;
//                    angleToKilobot = angleToKilobot - 180;
//                }
                angleToKilobot = path.angle() - orientDegrees;
                angleToKilobot = desNormAngle(angleToKilobot);

                //Compare with the orientation, don't use if it is not detected from the antenae
                //if (fabs(normAngle(angleToKilobot)) < 90){
                if ( angleToKilobot <= 90 || angleToKilobot >= 270 ){
                    //if (angleToKilobot < 0) angleToKilobot = angleToKilobot + 360;
                    if (floor(angleToKilobot/45) == 6) phToSend[0] = 1;
                    if (floor(angleToKilobot/45) == 7) phToSend[1] = 1;
                    if (floor(angleToKilobot/45) == 0) phToSend[2] = 1;
                    if (floor(angleToKilobot/45) == 1) phToSend[3] = 1;
                }
            }
        }
    }

    //Send the pheromone zones
    for(int i = 0; i < 4; i++){
        if (phToSend[i] != 0) {
            //For every number send 1,2,4 or 8 or the sum to know where are ph
            pheroZones = pow(2,i) + pheroZones;
        }
    }
    return pheroZones;
}

// generate virtual sensor readings & send to KB
void mykilobotenvironment::updateVirtualSensor(Kilobot kilobot_entity) {
    QPointF kPos = kilobot_entity.getPosition();
    float track_x = round(kPos.x()*MatrixSize_x/(ArenaX*1000));
    float track_y = round(kPos.y()*MatrixSize_y/(ArenaY*1000));
    kilobot_id kilobot_ID = kilobot_entity.getID();

    /******************************************************************/
    //Construct the message to send to the kilobots

    if(this->time - this->lastSent[kilobot_ID] > minTimeBetweenTwoMsg && !ongoingRuntimeIdentification){
        float angleToHome = 0;
        bool atHome = false;
        bool atFood = false;
        int foodSource = 0;
        foodClass tmpFood;
        int foodQuality = 0;
        kilobot_message message;

        for(int z = 0; z < NumFood; z++) {
            tmpFood = foodList.at(z);
            if((pow(kPos.x()-tmpFood.posX*1000,2)+pow(kPos.y()-tmpFood.posY*1000,2)) < pow(tmpFood.rad,2)){
                atFood = true;
                foodSource = z;
                foodQuality = tmpFood.quality;
                break;
            }
        }

        /****************************************/
        /****************************************/
        //Look if it is in the food and it is not going home
        if (atFood){
            atHome = false;
            //At food, save the food source, id and time
            if(hasFood[kilobot_ID] == 0){
                hasFood[kilobot_ID] = foodSource + 1;

                infoClass newInfo;
                newInfo.id = kilobot_ID;
                newInfo.source = foodSource + 1;
                newInfo.time = time;
                infoList.append(newInfo);

                /* modify food size */
//                foodClass newSource;
//                newSource = foodList.at(foodSource);
//                if (newSource.rad <= 10){
//                    //qDebug() << "Strange case in RAD 0: " << foodSource;
//                    newSource.rad = 0;
//                }else{
//                    newSource.rad = newSource.rad - 10;
//                    foodList.removeAt(foodSource);
//                    foodList.insert(foodSource, newSource);
//                }
            }

        }else{
//            atFood = false;
//            atHome = false;
            //Look if it is at home and is searching for home and add the food in case that it is
            if((pow(kPos.x()-HOME_X*1000,2)+pow(kPos.y()-HOME_Y*1000,2)) < pow(radHome,2)){
                //LOG << "Kilobot with id " << GetKilobotId(kilobot_entity) << " has deposited food" << std::endl;
                //Find home, go food
                atHome = true;
                //At home, save as home = 0, id and time
                if(hasFood[kilobot_ID] > 0){
                    hasFood[kilobot_ID] = 0;

                    infoClass newInfo;
                    newInfo.id = kilobot_ID;
                    newInfo.source = 0;
                    newInfo.time = time;
                    infoList.append(newInfo);
                    //radHome = radHome + 10;
                }
            }
    //        else{
    //            //Carrying food but led not change already
    //            if((hasFood[kilobot_ID] == 1) && (kilobot_entity.getLedColour() != BLUE)){
    //                atFood = true;
    //            }
    //        }
         }

        /******************************************************************/
        //Calculate the angle to home
        HomePos.setX(HOME_X*1000);
        HomePos.setY(HOME_Y*1000);
        QLineF path(kPos,HomePos);

        //Add the orientation
        double orientDegrees = qRadiansToDegrees(qAtan2(-kilobot_entity.getVelocity().y(), kilobot_entity.getVelocity().x()));
        angleToHome = path.angle() - orientDegrees;
        angleToHome = desNormAngle(angleToHome);


         lastSent[kilobot_ID] = this->time;
         message.id = kilobot_ID;
         message.type = 0;
         //10 bits (food, signal and angle)
         message.data = 0;

        //At food - Needs the quality instead of the pherozone
        if (atFood){
            //Save the quality from 7 to 4
            //newSource = foodList.at(foodSource);
            message.data = foodQuality;
            message.data = message.data << 4;
            message.data = message.data + 512;
            //qDebug() << "quality" << newSource.quality;
        }else{
            //If not carrying food, If not at home calculate the message of pheromone
            uint16_t msgZones = pheroZonesCalculation(track_x, track_y, orientDegrees);
            this->atPheromone[kilobot_ID] = (msgZones > 0);
            //Pherozones from 7 to 4 instead the quality
            message.data = msgZones;
           // if(msgZones > 0)      qDebug() << "msgZones" << msgZones;
            message.data = message.data << 4;
        }
        //At home
        if (atHome) message.data =  message.data + 256;

        //Angle to home in the 4 LSB
        angleToHome = round(angleToHome/45);
        if (angleToHome == 8) angleToHome = 0;
        message.type = angleToHome;
        //message.data = angleToHome + message.data;
        //if (kilobot_entity.getLedColour() == BLUE){
//        if (kilobot_entity.getID() == 0){
//            qDebug() << "message.data(" << kilobot_entity.getID() << "):" << message.data << " and message.type=" << message.type << " -> angleToHome;" << angleToHome;
//        }
        emit transmitKiloState(message);

    }

    /******************************************************************/
    // Deposit pheromone if led is blue
    if ( (this->ongoingRuntimeIdentification && this->isPrinting[kilobot_ID])
        || (this->hasFood[kilobot_ID] > 0 && kilobot_entity.getLedColour() == BLUE && !this->ongoingRuntimeIdentification) ){
        //qDebug() << "Robot " << kilobot_ID << " is blue.";
        floorMatrix[I2I(track_x,track_y)] = qMin( floorMatrix[I2I(track_x,track_y)] + pheromone_amount, maxPheromoneValue);
//        if(floorMatrix[int(track_x)][int(track_y)] > maxPheromoneValue){
//            floorMatrix[int(track_x)][int(track_y)] = maxPheromoneValue;
//        }
        this->isPrinting[kilobot_ID] = true;
    } else {
        this->isPrinting[kilobot_ID] = false;
    }
}




