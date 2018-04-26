/** Author: Anna Font Llenas and Andreagiovanni Reina a.reina@sheffield.ac.uk
 *  Copyright University of Sheffield, 2018
 *  If you use this code for scientific experiment, please cite:
 *  A. Font Llenas et al. 2018 in ANTS 2018
 */

#ifndef TESTLIB_H
#define TESTLIB_H

#include "global.h"

//
#include <QObject>
#include <QFile>
//#include <QTextStream>

// Project includes
#include "kilobot.h"
#include "kilobotexperiment.h"
#include "kilobotenvironment.h"
#include "Experiment1Env.h"

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <QObject>
#include <QFile>
#include <QList>
#include <QTableWidget>
#include <QSpinBox>
#include <QFormLayout>
#include <iterator>
#include <vector>


#include <QPushButton>
#include <QCheckBox>
#include <QScrollBar>
#include <QSlider>
#include <QGroupBox>
#include <QFormLayout>
#include <QLabel>
#include <QFrame>
#include <QtMath>
#include <QElapsedTimer>

using namespace cv;

class KiloLog {
public:
    // constructors
    KiloLog() {}
    KiloLog(kilobot_id id, QPointF pos, double rot, kilobot_colour col) : id(id), position(pos), orientation(rot), colour(col) {}

    // methods
    void updateAllValues(kilobot_id id, QPointF pos, double rot, kilobot_colour col){
        this->id = id;
        this->position = pos;
        this->orientation = rot;
        this->colour = col;
    }
    void setPos(QPointF pos){
        this->position = pos;
    }
    void setOrientation(double rot){
        this->orientation = rot;
    }
    void setCol(kilobot_colour col){
        this->colour = col;
    }

    // variables
    kilobot_id id;
    QPointF position;
    double orientation;
    kilobot_colour colour;
};

class EXPERIMENT1EXPSHARED_EXPORT mykilobotexperiment : public KilobotExperiment
{
    Q_OBJECT
public:
    mykilobotexperiment();
    virtual ~mykilobotexperiment() {}

    QWidget * createGUI();
    QVBoxLayout * lay = new QVBoxLayout;

signals:
    void errorMessage(QString);
//    void setExptImage(QPixmap);

public slots:
    void initialise(bool);
    void run();
    void stopExperiment();

//  void setupExperiment();
    void toggleSaveImages(bool toggle) { saveImages = toggle; }
    void toggleLogExp(bool toggle) {logExp = toggle;}
    void radio1Selected();
    void radio2Selected();
    void radio3Selected();
    void radio4Selected();
    void addFood();
    QColor GetFloorColor(int x, int y);
    void setdiffckb(int val);
    void setFOODX(double val);
    void setFOODY(double val);
    void setQuality(int val);
    void setIndex(int val);
    inline void setDifRate(double val) { pheroEnv.diffusion_rate = (float)val; }
    inline void setEvapRate(double val) { pheroEnv.evaporation_rate = (float)val; }
    inline void setQuantityRate(int val) { pheroEnv.pheromone_amount = val; }
    inline void setHOMEX(double val) { pheroEnv.HOME_X = (float)val; }
    inline void setHOMEY(double val) { pheroEnv.HOME_Y = (float)val; }
    void setIndex(bool toggle);

private:
    void updateKilobotState(Kilobot kilobotCopy);
    void setupInitialKilobotState(Kilobot kilobotCopy);

    //
    void setupEnvironments();
    void plotEnvironment();
    void printCollectedFood();

    //
    mykilobotenvironment pheroEnv;

    // logging variables
    bool saveImages;
    int savedImagesCounter;
    bool logExp;
    QFile log_file;
    QString log_filename_prefix="log_phero";
    QTextStream log_stream;
    QString log_matrix_filename_prefix = "log_matrix";
    QTextStream log_matrix_stream;
    QFile log_matrix_file;

    //GUI objects
    QDoubleSpinBox * diff_spin;
    QDoubleSpinBox * evap_spin;
    QSpinBox * amount_spin;
    QDoubleSpinBox * HOMEX_spin;
    QDoubleSpinBox * HOMEY_spin;
    QDoubleSpinBox * FOODX_spin[5];
    QDoubleSpinBox * FOODY_spin[5];
    QSpinBox * Quality_spin[5];
    QGroupBox * sourceFood[5];
    QCheckBox * EditSource[5];
    int oldNumFood;
    QList<foodClass>::iterator FoodSpinindex;

    //kilo objects
    QVector < kilobot_id >  allKiloIDs;
    QVector <KiloLog> allKilos;

};


#endif // TESTLIB_H
