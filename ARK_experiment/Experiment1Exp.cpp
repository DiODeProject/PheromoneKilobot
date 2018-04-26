/** Author: Anna Font Llenas and Andreagiovanni Reina a.reina@sheffield.ac.uk
 *  Copyright University of Sheffield, 2018
 *  If you use this code for scientific experiment, please cite:
 *  A. Font Llenas et al. 2018 in ANTS 2018
 */

#include "Experiment1Exp.h"
#include "Experiment1Env.h"
#include <QDebug>
#include <QThread>

// widgets
#include <QPushButton>
#include <QCheckBox>
#include <QScrollBar>
#include <QSlider>
#include <QGroupBox>
#include <QFormLayout>
#include <QLabel>
#include <QFrame>
#include <QtMath>
#include <QRadioButton>
#include <QPainter>
#include <QList>
#include <iterator>
#include <QSignalMapper>
#include <QFile>

#define I2I(x,y) int(x)*pheroEnv.MatrixSize_x+int(y)

// return pointer to interface!
// mykilobotexperiment can and should be completely hidden from the application
extern "C" EXPERIMENT1EXPSHARED_EXPORT KilobotExperiment *createExpt()
{
    return new mykilobotexperiment();
}

mykilobotexperiment::mykilobotexperiment() {

    // setup the environments here
    connect(&pheroEnv,SIGNAL(transmitKiloState(kilobot_message)), this, SLOT(signalKilobotExpt(kilobot_message)));

    this->serviceInterval = 100; // timestep in ms

}



QWidget * mykilobotexperiment::createGUI() {
    QFrame * frame = new QFrame;
    frame->setLayout(lay);

    QCheckBox * saveImages_ckb = new QCheckBox("Record experiment");
    saveImages_ckb->setChecked(false);
    lay->addWidget(saveImages_ckb);
    toggleSaveImages(saveImages_ckb->isChecked());

    QCheckBox * logExp_ckb = new QCheckBox("Log experiment");
    logExp_ckb->setChecked(true);
    lay->addWidget(logExp_ckb);
    toggleLogExp(logExp_ckb->isChecked());

    //Create a box for the pheromone parameters
     QGroupBox * pheroParams = new QGroupBox(tr("Pheromone:"));
     QFormLayout * layout2 = new QFormLayout;
     pheroParams->setLayout(layout2);

     //Diffusion
     diff_spin = new QDoubleSpinBox();
     diff_spin->setMinimum(0);
     diff_spin->setMaximum(0.1);
     diff_spin->setSingleStep(0.01);
     diff_spin->setValue(pheroEnv.diffusion_rate);
     layout2->addRow(new QLabel(tr("Diffusion:")), diff_spin);

     //Evaporation
     evap_spin = new QDoubleSpinBox();
     evap_spin->setMinimum(0);
     evap_spin->setMaximum(0.1);
     evap_spin->setSingleStep(0.001);
     evap_spin->setDecimals(3);
     evap_spin->setValue(pheroEnv.evaporation_rate);
     layout2->addRow(new QLabel(tr("Evaporation:")), evap_spin);
     lay->addWidget(pheroParams);

     //Amount
     amount_spin = new QSpinBox();
     amount_spin->setMinimum(0);
     amount_spin->setMaximum(2000);
     //amount_spin->setDecimals(0);
     amount_spin->setValue(pheroEnv.pheromone_amount);
     layout2->addRow(new QLabel(tr("Quantity:")), amount_spin);
     lay->addWidget(pheroParams);

     //Position home and food
     QGroupBox * sourcePosition = new QGroupBox(tr("Home Position:"));
     QFormLayout * layout3 = new QFormLayout;
     sourcePosition->setLayout(layout3);

     //Home X
     HOMEX_spin = new QDoubleSpinBox();
     HOMEX_spin->setMinimum(0);
     HOMEX_spin->setMaximum(2);
     HOMEX_spin->setSingleStep(0.1);
     HOMEX_spin->setDecimals(3);
     HOMEX_spin->setValue(pheroEnv.HOME_X);
     layout3->addRow(new QLabel(tr("Home X:")), HOMEX_spin);

     //Home Y
     HOMEY_spin = new QDoubleSpinBox();
     HOMEY_spin->setMinimum(0);
     HOMEY_spin->setMaximum(2);
     HOMEY_spin->setSingleStep(0.1);
     HOMEY_spin->setDecimals(3);
     HOMEY_spin->setValue(pheroEnv.HOME_Y);
     layout3->addRow(new QLabel(tr("Home Y:")), HOMEY_spin);

     //Num of food soure
     QGroupBox *groupBox = new QGroupBox();
     groupBox->setTitle("Select food source:");
     QRadioButton *radio1 = new QRadioButton(tr("1"));
     QRadioButton *radio2 = new QRadioButton(tr("2"));
     QRadioButton *radio3 = new QRadioButton(tr("3"));
     QRadioButton *radio4 = new QRadioButton(tr("4"));
     QHBoxLayout *vbox = new QHBoxLayout;
     vbox->addWidget(radio1);
     vbox->addWidget(radio2);
     vbox->addWidget(radio3);
     vbox->addWidget(radio4);
     vbox->addStretch(1);
     groupBox->setLayout(vbox);
     connect(radio1,SIGNAL(clicked()),this,SLOT(radio1Selected()));
     connect(radio2,SIGNAL(clicked()),this,SLOT(radio2Selected()));
     connect(radio3,SIGNAL(clicked()),this,SLOT(radio3Selected()));
     connect(radio4,SIGNAL(clicked()),this,SLOT(radio4Selected()));

     lay->addWidget(sourcePosition);
     lay->addWidget(groupBox);

     connect(diff_spin, SIGNAL(valueChanged(double)),this, SLOT(setDifRate(double)));
     connect(evap_spin, SIGNAL(valueChanged(double)),this, SLOT(setEvapRate(double)));
     connect(amount_spin, SIGNAL(valueChanged(int)),this, SLOT(setQuantityRate(int)));
     connect(HOMEX_spin, SIGNAL(valueChanged(double)),this, SLOT(setHOMEX(double)));
     connect(HOMEY_spin, SIGNAL(valueChanged(double)),this, SLOT(setHOMEY(double)));
     connect(saveImages_ckb, SIGNAL(toggled(bool)),this, SLOT(toggleSaveImages(bool)));
     connect(logExp_ckb, SIGNAL(toggled(bool)),this, SLOT(toggleLogExp(bool)));
     connect(this,SIGNAL(destroyed(QObject*)), lay, SLOT(deleteLater()));
     return frame;
}

void mykilobotexperiment::initialise(bool isResume) {

    // Generate Environments:
    setupEnvironments();

    // Initialize Kilobot States:
    if (!isResume) {
        // init stuff
        emit getInitialKilobotStates();
    } else {
        // probably nothing
    }

    //Pos = position, ROT = orientation, LED = color
   emit setTrackingType(POS | ROT | LED);

    QThread::currentThread()->setPriority(QThread::HighestPriority);

    savedImagesCounter = 0;
    this->time = 0;

    // Init Log File operations
    if (logExp){
        //Open file for position / food
        if (log_file.isOpen()){
            log_file.close(); // if it was open I close and re-open it (erasing old content!! )
        }
        QString log_filename = log_filename_prefix + ".txt";
//        QString log_filename = log_filename_prefix + "_" + QDate::currentDate().toString("yyMMdd") + "_" + QTime::currentTime().toString("hhmmss") + ".txt";
        log_file.setFileName( log_filename );
        if ( !log_file.open(QIODevice::WriteOnly) ) { // open file
            qDebug() << "ERROR(!) in opening file" << log_filename;
        } else {
            qDebug () << "Log file" << log_file.fileName() << "opened.";
            log_stream.setDevice(&log_file);
        }

        //Open file for matrix
        QString filenameMatrix = log_matrix_filename_prefix + ".txt";
        log_matrix_file.setFileName(filenameMatrix);
        if ( !log_matrix_file.open(QIODevice::WriteOnly) ) { // open file
            qDebug() << "ERROR(!) in opening file" << filenameMatrix;
        } else {
            qDebug () << "Log file" << log_matrix_file.fileName() << "opened.";
            log_matrix_stream.setDevice(&log_matrix_file);
        }
    }

    if (saveImages) {
        emit saveImage(QString("phero_%1.jpg").arg(savedImagesCounter++, 5,10, QChar('0')));
    }
    /*********** Write Position file ************************/
    if (logExp){
        log_stream << this->time;
        for (int i = 0; i < allKiloIDs.size(); ++i){
            kilobot_id kID = allKiloIDs[i];
            log_stream << "\t" << kID << "\t" << allKilos[kID].position.x() << "\t" << allKilos[kID].position.y() << "\t" << allKilos[kID].orientation << "\t" << allKilos[kID].colour << "\t" << pheroEnv.hasFood[kID] << "\t" << pheroEnv.atPheromone[kID];
        }
        log_stream << endl;
    }


    // clear old drawings (e.g., from ID-identification)
    clearDrawings();
}

void mykilobotexperiment::setFOODX(double foodX) {

    foodClass newSource;
    //Return the value from first point to the iterator
    int index = std::distance(pheroEnv.foodList.begin(), FoodSpinindex);
    newSource = pheroEnv.foodList.at(index);
    newSource.posX = foodX;
    pheroEnv.foodList.erase(FoodSpinindex);
    pheroEnv.foodList.insert(index, newSource);

}
void mykilobotexperiment::setFOODY(double foodY) {

    foodClass newSource;
    //Return the value from first point to the iterator
    int index = std::distance(pheroEnv.foodList.begin(), FoodSpinindex);
    newSource = pheroEnv.foodList.at(index);
    newSource.posY = foodY;
    pheroEnv.foodList.erase(FoodSpinindex);
    pheroEnv.foodList.insert(index, newSource);
}
void mykilobotexperiment::setQuality(int quality) {

    foodClass newSource;
    //Return the value from first point to the iterator

    int index = std::distance(pheroEnv.foodList.begin(), FoodSpinindex);
    newSource = pheroEnv.foodList.at(index);
    newSource.quality = quality;
    pheroEnv.foodList.erase(FoodSpinindex);
    pheroEnv.foodList.insert(index, newSource);
}

void mykilobotexperiment::addFood() {
    pheroEnv.foodList.clear();
    int initQuality[4] = {10,5,8,3};
    for(int i = 0; i < oldNumFood; i++){
        delete(sourceFood[i]);
    }

    for (int index = 0; index < pheroEnv.NumFood; index++) {
        foodClass newSource;

        newSource.posX = 0.5;
        newSource.posY = 1.5;
        newSource.rad = 100;

//        if (index > 1) newSource.posX = 1.5;
//        if (index%2 > 0) newSource.posY = 1.5;

        if (index == 0){
            newSource.posX = 1.0-0.424;
            newSource.posY = 1.0-0.424;
        } else
        if (index == 1){
            newSource.posX = 1.707;
            newSource.posY = 1.707;
//            newSource.posX = 1.424;
//            newSource.posY = 1.424;
        } else
        if (index == 2){
//            newSource.posX = 1.566;
//            newSource.posY = 1.0-0.566;
            newSource.posX = 1.707;
            newSource.posY = 1.0-0.707;
        } else
        if (index == 3){
//            newSource.posX = 1.0-0.353;
//            newSource.posY = 1.353;
            newSource.posX = 1.0-0.707;
            newSource.posY = 1.707;
        }

        newSource.quality = initQuality[index];

       //Position food
        sourceFood[index] = new QGroupBox();
        sourceFood[index]->setTitle("Food Source "+QString::number(index+1));
        QFormLayout * layout4 = new QFormLayout;
        sourceFood[index]->setLayout(layout4);

        EditSource[index] = new QCheckBox("Edit Source");
        EditSource[index]->setObjectName("Edit Source" + index);
        EditSource[index]->setChecked(false);
        layout4->addWidget(EditSource[index]);
        //toggleLogExp(EditSource[index]->isChecked());

        //Food X
        FOODX_spin[index] = new QDoubleSpinBox();
        FOODX_spin[index]->setMinimum(0);
        FOODX_spin[index]->setMaximum(2);
        FOODX_spin[index]->setSingleStep(0.1);
        FOODX_spin[index]->setDecimals(3);
        FOODX_spin[index]->setValue(newSource.posX);
        layout4->addRow(new QLabel(tr("Food X:")), FOODX_spin[index]);

        //Food Y
        FOODY_spin[index] = new QDoubleSpinBox();
        FOODY_spin[index]->setMinimum(0);
        FOODY_spin[index]->setMaximum(2);
        FOODY_spin[index]->setSingleStep(0.1);
        FOODY_spin[index]->setDecimals(3);
        FOODY_spin[index]->setValue(newSource.posY);
        layout4->addRow(new QLabel(tr("Food Y:")), FOODY_spin[index]);
        lay->addWidget(sourceFood[index]);

        //Quality
        Quality_spin[index] = new QSpinBox();
        Quality_spin[index]->setMinimum(0);
        Quality_spin[index]->setMaximum(10);
        Quality_spin[index]->setValue(newSource.quality);
        layout4->addRow(new QLabel(tr("Quality:")), Quality_spin[index]);
        lay->addWidget(sourceFood[index]);

        pheroEnv.foodList.insert(index,newSource);


        FOODX_spin[index]->setEnabled( false );
        FOODY_spin[index]->setEnabled( false );
        Quality_spin[index]->setEnabled( false );

        QSignalMapper* signalMapper = new QSignalMapper (this);
        connect (EditSource[index], SIGNAL(toggled(bool)), signalMapper, SLOT(map())) ;
        signalMapper -> setMapping (EditSource[index], index);
        connect (signalMapper, SIGNAL(mapped(int)), this, SLOT(setIndex(int))) ;
        connect(FOODX_spin[index] , SIGNAL(valueChanged(double)), this, SLOT(setFOODX(double)));
        connect(FOODY_spin[index] , SIGNAL(valueChanged(double)),this, SLOT(setFOODY(double)));
        connect(Quality_spin[index] , SIGNAL(valueChanged(int)),this, SLOT(setQuality(int)));

    }


}

void mykilobotexperiment::radio1Selected() {
    oldNumFood = pheroEnv.NumFood;
    pheroEnv.NumFood = 1;
    addFood();
}
void mykilobotexperiment::radio2Selected() {
    oldNumFood = pheroEnv.NumFood;
    pheroEnv.NumFood = 2;
    addFood();
}
void mykilobotexperiment::radio3Selected() {
    oldNumFood = pheroEnv.NumFood;
    pheroEnv.NumFood = 3;
    addFood();
}
void mykilobotexperiment::radio4Selected() {
    oldNumFood = pheroEnv.NumFood;
    pheroEnv.NumFood = 4;
    addFood();
}

void mykilobotexperiment::setIndex(int val) {
    FoodSpinindex = pheroEnv.foodList.begin();
    std::advance (FoodSpinindex, val);
    FOODX_spin[val]->setEnabled( true );
    FOODY_spin[val]->setEnabled( true );
    Quality_spin[val]->setEnabled( true );
    for (int i = 0; i < pheroEnv.NumFood; i++){
        if(i != val){
            FOODX_spin[i]->setEnabled( false );
            FOODY_spin[i]->setEnabled( false );
            Quality_spin[i]->setEnabled( false );
            EditSource[i]->setChecked(false);
        }
    }

}


void mykilobotexperiment::stopExperiment() {

    /*********** Close Log file ************************/
    if (log_file.isOpen()){
        qDebug() << "Closing file" << log_file.fileName();
        log_file.close();
    }

    /*********** Close Phero Matrix file ************************/
    if (log_matrix_file.isOpen()){
        qDebug() << "Closing file" << log_matrix_file.fileName();
        log_matrix_file.close();
    }


    /*********** Write Food collected file ************************/

    if(!pheroEnv.infoList.empty()){
        QString filename = "log_food_collected.txt";
        QFile file(filename);
        QTextStream out(&file);
        if(file.open(QIODevice::WriteOnly)){
            out << "ID" << "\t" << "Source" << "\t" << "Time" << endl;
            while(!pheroEnv.infoList.empty()){
                out << pheroEnv.infoList.constFirst().id << "\t" << pheroEnv.infoList.constFirst().source << "\t" << pheroEnv.infoList.constFirst().time << endl;
                pheroEnv.infoList.removeFirst();
            }
            file.close();
        }
    }


    /*********** Delete dynamic arrays **************************/
    //Delete the NewPhero and the floorMatrix
    free(pheroEnv.floorMatrix);

}

void mykilobotexperiment::run() {
    this->time += 0.1; // 100 ms in sec
    // Update Environment
    pheroEnv.time = (float)time;
    pheroEnv.ongoingRuntimeIdentification = this->runtimeIdentificationLock;
    // update and save matrix pheromone once every matrix_update_time
    if(time >= pheroEnv.last_matrix_update + pheroEnv.matrix_update_time){
        //qDebug() << "Updating matrix" << time;
        pheroEnv.update();
    }

    //Update updatekiloState
    emit updateKilobotStates();

    int intTime = qRound(time*10.0);

    if ( (intTime+2) % 10 == 0) { // every 1s
        //Clear current environment
        clearDrawings();
        clearDrawingsOnRecordedImage();

        //Plot environment
        plotEnvironment();
    }

    if (intTime % 5 == 0) { // every 0.5s
        if (saveImages) {
            emit saveImage(QString("phero_%1.jpg").arg(savedImagesCounter++, 5,10, QChar('0')));
        }
        if (logExp){
            /*********** Write Phero Matrix file ************************/
            for (int row = 0; row < pheroEnv.MatrixSize_x; ++row) {
                for (int column = 0; column < pheroEnv.MatrixSize_y; ++column) {
                    log_matrix_stream << pheroEnv.floorMatrix[I2I(row,column)] << "\t";
                }
                log_matrix_stream << endl;
            }
            log_matrix_stream << endl;
            if (qRound(time*10.0) % 50 == 0) {
                log_matrix_stream.flush();
            }

            /*********** Write Position file ************************/
            log_stream << this->time ;
            for (int i = 0; i < allKiloIDs.size(); ++i){
                kilobot_id kID = allKiloIDs[i];
                log_stream << "\t" << kID << "\t" << allKilos[kID].position.x() << "\t" << allKilos[kID].position.y() << "\t" << allKilos[kID].orientation << "\t" << allKilos[kID].colour << "\t" << pheroEnv.hasFood[kID] << "\t" << pheroEnv.atPheromone[kID];
            }
            log_stream << endl;
        }
    }

}

// Setup the Initial Kilobot Environment:
//   This is run once for each kilobot after emitting getInitialKilobotStates() signal.
//   This assigns kilobots to an environment.
void mykilobotexperiment::setupInitialKilobotState(Kilobot kilobot_entity) {
    // Assigns all kilobots to environment pheroEnv:
    this->setCurrentKilobotEnvironment(&pheroEnv);

    kilobot_id kID = kilobot_entity.getID();

    // create a necessary lists and variables to get the right timing for messaging
    if (kID > allKilos.size()-1){
        pheroEnv.lastSent.resize(kID+1);
        pheroEnv.hasFood.resize(kID+1);
        pheroEnv.isPrinting.resize(kID+1);
        pheroEnv.atPheromone.resize(kID+1);
        allKilos.resize(kID+1);
    }

    pheroEnv.lastSent[kID] = pheroEnv.minTimeBetweenTwoMsg; // robots get different values in lastSent to have asynchrony
    pheroEnv.hasFood[kID] = 0; // initially robots carry no food
    pheroEnv.atPheromone[kID] = false; // initially robots at not at pheromone
    pheroEnv.isPrinting[kID] = false; // initially robots are not printing pheromone

    KiloLog kLog(kID, kilobot_entity.getPosition(), 0, kilobot_entity.getLedColour());
    allKilos[kID] = kLog;
    if (!allKiloIDs.contains(kID)) allKiloIDs.append(kID);

    double timeForAMessage = 0.05; // 50ms each message
    pheroEnv.minTimeBetweenTwoMsg = allKiloIDs.size()*timeForAMessage/2.8;
    pheroEnv.lastSent[kID] = pheroEnv.minTimeBetweenTwoMsg;
    //qDebug() << "Min time between two messages is" << pheroEnv.minTimeBetweenTwoMsg;

}

// run once for each kilobot after emitting updateKilobotStates() signal
void mykilobotexperiment::updateKilobotState(Kilobot kilobotCopy) {
    // update values for logging
    if (logExp && (qRound(time*10.0) % 5 == 0) ){
        kilobot_id kID = kilobotCopy.getID();
        kilobot_colour kCol = kilobotCopy.getLedColour();
        QPointF kPos = kilobotCopy.getPosition();
        double kRot = qRadiansToDegrees(qAtan2(-kilobotCopy.getVelocity().y(), kilobotCopy.getVelocity().x()));
        allKilos[kID].updateAllValues(kID, kPos, kRot, kCol);
    }
}


// Setup Environment:
void mykilobotexperiment::setupEnvironments( ) {
    pheroEnv.reset();
}


QColor mykilobotexperiment::GetFloorColor(int track_x, int track_y){

   QColor floorColour = Qt::white;
   //Paint the pheromone
   //Darker if more pheromone
   int pheroColourLimit = 1000;
   if (pheroEnv.floorMatrix[I2I(track_x,track_y)] > pheroColourLimit) {
       floorColour=QColor(0,0,0);
   } else {
       int normVal = pheroEnv.floorMatrix[I2I(track_x,track_y)]*255/pheroColourLimit;
       floorColour=QColor(max(0,230-normVal), 240, 255-normVal*0.7);
   }
   return floorColour;
}

void mykilobotexperiment::printCollectedFood(){
    //In the list, right position is kilosize*time
    for(int i = 0; i < allKiloIDs.size(); i++){
        //If kilobot carrying food lets print the food
        if(pheroEnv.hasFood[allKiloIDs.at(i)] > 0){
            KiloLog kLog = allKilos.at(allKiloIDs.at(i));
            int shiftX =  (kLog.orientation < 180)? -15 : 15;
            int shiftY =  (kLog.orientation > 90 && kLog.orientation < 270)? -15 : 15;
            drawCircle(QPointF(kLog.position.x()+shiftX, kLog.position.y()+shiftY),6, QColor(Qt::yellow), 12, "", true);
        }
    }
}

// Plot Environment on frame:
void mykilobotexperiment::plotEnvironment() {
    QColor floorColour = Qt::white;
    foodClass newSource;
    float pheroNewX;
    float pheroNewY;
    float radPhero = pheroEnv.ArenaX*2000/pheroEnv.MatrixSize_x;

    //Print home
    drawCircle(QPointF(pheroEnv.HOME_X*1000,pheroEnv.HOME_Y*1000),pheroEnv.radHome, QColor(Qt::green), 3, "", false);

    //Print food
    for(int z = 0; z < pheroEnv.NumFood; z++) {
        newSource = pheroEnv.foodList.at(z);
        if (newSource.rad > 10){
            drawCircle(QPointF(newSource.posX*1000,newSource.posY*1000),newSource.rad, QColor(round(24.5*(10-newSource.quality/1.8)),10,5), 3, "", false);
        }
    }

    //Actualize the matrix
    for (int i = 0; i < pheroEnv.MatrixSize_x; i+=2){
         for (int j = 0; j < pheroEnv.MatrixSize_y; j+=2){
             //Print pheromone
             if(pheroEnv.floorMatrix[I2I(i,j)] > 0) {
                 floorColour = GetFloorColor(i, j);
                 pheroNewX = round(i*pheroEnv.ArenaX*1000/pheroEnv.MatrixSize_x);
                 pheroNewY = round(j*pheroEnv.ArenaY*1000/pheroEnv.MatrixSize_y);
                 drawCircle(QPointF(pheroNewX,pheroNewY),radPhero, QColor(floorColour), 3, "", true);
             }
        }
    }
    printCollectedFood();
 }


