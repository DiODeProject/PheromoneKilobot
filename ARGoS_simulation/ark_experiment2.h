#ifndef ARK_EXPERIMENT2_H
#define ARK_EXPERIMENT2_H

namespace argos {
    class CSpace;
    class CRay3;
    class CFloorEntity;
    class CSimulator;
}


#include <math.h>
// basic file operations
#include <iostream>
#include <fstream>

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/logging/argos_log.h>
//#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>

#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_medium.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_default_actuator.h>

//kilobot messaging
#include <argos3/plugins/robots/kilobot/control_interface/kilolib.h>
#include <argos3/plugins/robots/kilobot/control_interface/message_crc.h>
#include <argos3/plugins/robots/kilobot/control_interface/message.h>

#include <array>
#include <list>
#include <iostream>
#include <vector>

using namespace std;
using namespace argos;

class CArk : public CLoopFunctions { //, public CQTOpenGLUserFunctions {
    
public:
    
    CArk();
    virtual ~CArk();
    virtual void Init(TConfigurationNode& t_tree);// Can be used to add things from .argos file
    virtual void Reset();
    virtual void Destroy();
    virtual void PreStep();
    virtual void PostStep();
    
    // Create the OHC entity
    void CreateOhcCommunicationEntities(unsigned int number_of_comm_entities);
    
    // Get a Vector of all the Kilobots in the space
    void GetKilobotsEntities();
    
    
    // Get the message to send to a Kilobot according to its position
    void Get_message_receive(CKilobotEntity *kilobot_entity, message_t &message);
    
    // Get the message to send to a Kilobot according to its position
    void Get_message_to_send(CKilobotEntity *kilobot_entity, message_t &message);
    
    // Get the position of one Kilobot
    CVector2 GetKilobotPosition(CKilobotEntity *kilobot_entity);
    
    // Get the orientation of one Kilobot
    CRadians GetKilobotOrientation(CKilobotEntity *kilobot_entity);
    
    // Get the kilobot ID
    UInt16 GetKilobotId(CKilobotEntity *kilobot_entity);
    
    // Get the kilobot LED color
    CColor GetKilobotLedColor(CKilobotEntity *kilobot_entity);
    
    // Set the floor color (It is specified in the .argos file that the loop function is the source a floor entity)
    virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
    
    // Get the options from the xml file
    void GetOptions(TConfigurationNode& t_tree);
    
    // Add an option to the options vector
    void AddOption(TConfigurationNode& t_node);
        
    // converte a number to a color
    CColor inttoccolor(int color_id);
    
private:
    
    //Calculate the evaporation and diffusion for each cell
    void updatePheromoneMatrix();
    
    void write_file();

    // A reference to the simulator
    CSimulator& m_cSimulator;
    
    // A reference to the space
    //CSpace& m_cSpace;
    
    // Space size
    CVector3 m_vSpaceSize;

    // Padding of the space to shift the index and access the floorMatrix
    CVector3 m_vSpaceLimits;
    
    // Cells per metre
    int m_iCellsPerMetre;

    //The OHC embodied entity
    CEmbodiedEntity m_ohcEmbodiedEntity;
    
    //The OHC communication Anchor
    SAnchor& m_cCommAnchor;
    
    //List of the OHC communication entities
    typedef std::vector<CKilobotCommunicationEntity> KCVector;
    KCVector m_ohcCommunicationEntities;
    
    //List of the Kilobots in the space
    typedef std::vector<CKilobotEntity*> KVector;
    KVector m_KilobotsEntities;
    
    // List of the messages sent by communication entities
    typedef std::vector<message_t> KMVector;
    KMVector m_messages;
    
    //A flag for a kilbot needs a message to be sent or not
    bool m_tx_flag;
    
    //Counters for messages and data acquizition
    unsigned int m_messaging_counter, m_floor_counter;
    unsigned int m_update_matrix_timesteps;
    
    //output file name
    std::string m_strOutput;
    
    //Radius of the circles
    std::vector<int> hasFood;
    std::map<UInt16, int> collected;
    std::vector<int> onPath;
    float phToSend[210][4];
    
    //options vector
    struct FoodClass{
    		int id;
        int quality;
        CVector2 position;
        Real radius;
        UInt8 colour;
    };

    	std::vector<FoodClass> foodVector;

    int MatrixSize_x;
    int MatrixSize_y;
    float *floorMatrix;
//    std:vector< std::vector< int > > = floorMatrix;
    float maxPheroLimit;

    std::string m_strOutputFilename;
    
    float evaporation_rate, diffusion_rate, pheromone_amount;
    
//    std::vector< CVector3 >  kb_positions;
    
};




#endif
