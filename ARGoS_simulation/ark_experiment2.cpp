/** Author: Anna Font Llenas and Andreagiovanni Reina a.reina@sheffield.ac.uk
 *  Copyright University of Sheffield, 2018
 *  If you use this code for scientific experiment, please cite:
 *  A. Font Llenas et al. 2018 in ANTS 2018
 */

#include "ark_experiment2.h"
#include <limits>
//#include <argos3/plugins/robots/kilobot/simulator/kilobot_measures.h>
#define I2I(x,y) int(x)*MatrixSize_x+int(y)

/****************************************/
/****************************************/
CArk::CArk(): CLoopFunctions(),m_cSimulator(GetSimulator()), m_iCellsPerMetre(100), m_ohcEmbodiedEntity(CEmbodiedEntity(NULL, "ohc_body", CVector3(0,0,1), CQuaternion())),
m_cCommAnchor(m_ohcEmbodiedEntity.AddAnchor("comm", CVector3(0,0,0))),m_tx_flag(false), m_messaging_counter(0),m_floor_counter(0), m_update_matrix_timesteps(0),
MatrixSize_x(0), MatrixSize_y(0), floorMatrix(0), maxPheroLimit(std::numeric_limits<float>::max()), evaporation_rate(0.003), diffusion_rate(0.01), pheromone_amount(75){
}
/****************************************/
/****************************************/
CArk::~CArk(){
}
/****************************************/
/****************************************/
void CArk::Init(TConfigurationNode& t_node) {
    
	m_update_matrix_timesteps = 10;
    
    // Get the kilobots entities
    GetKilobotsEntities();

    //Create the OHC comm entities
    CreateOhcCommunicationEntities(m_KilobotsEntities.size());
    
    // Get the output datafile name and open it
    GetNodeAttributeOrDefault(t_node, "datafilename", m_strOutputFilename, std::string("file.out") );
    
    // Get the special areas in the space
    GetOptions(t_node);
    
    //Initialize global variables
    hasFood.resize(m_KilobotsEntities.size(),0);
//    collected.resize(foodVector.size(),0);
    for(UInt8 i = 0; i < foodVector.size(); ++i){
    		if (foodVector.at(i).id == 0) continue; // skip home
    		collected[foodVector.at(i).id] = 0;
    }
    onPath.resize(foodVector.size(), 0);

    // Get number of pheromone matrix cells per metre
    GetNodeAttribute(t_node,"cells_per_metre",m_iCellsPerMetre);
    
    m_vSpaceSize = CSimulator::GetInstance().GetSpace().GetArenaSize();
    MatrixSize_x = m_vSpaceSize.GetX()*m_iCellsPerMetre;
    MatrixSize_y = m_vSpaceSize.GetY()*m_iCellsPerMetre;
    m_vSpaceLimits = CSimulator::GetInstance().GetSpace().GetArenaLimits().GetMin();
    
    // Configuring the pheromone parameters
    TConfigurationNode pheroNode = GetNode(t_node,"pheromone_params");
    GetNodeAttribute(pheroNode,"evaporation_rate",evaporation_rate);
    GetNodeAttribute(pheroNode,"diffusion_rate",diffusion_rate);
    GetNodeAttribute(pheroNode,"pheromone_amount",pheromone_amount);

    // Write the parameters
    //LOG << "maxPheroLimit rate is " << maxPheroLimit << std::endl;
    LOG << "Evap rate is " << evaporation_rate << std::endl;
    LOG << "Diff rate is " << diffusion_rate << std::endl;
    LOG << "Ph amount is " << pheromone_amount << std::endl;

    LOG << "There are " << foodVector.size()-1 << " option" << ((foodVector.size()>2)?"s":"") << endl;
    for (UInt8 i =0; i< foodVector.size(); ++i){
    		if (foodVector.at(i).id==0)
    			LOG << "Home is located at ";
    		else
    			LOG << "Option #" << foodVector[i].id << " quality: " << foodVector[i].quality << " is located at ";
    		LOG << foodVector[i].position << endl;
    	}

    // Create the FloorMatrix
    floorMatrix = (float*) malloc(MatrixSize_x*MatrixSize_y*sizeof(float));
    memset(floorMatrix, 0, MatrixSize_x*MatrixSize_y*sizeof(float));
    LOG << "Floor Matrix size: " << MatrixSize_x << " x " << MatrixSize_y << endl;
    // Initialize the FloorMatrix
//    for(int i = 0; i < MatrixSize_x; i++){
//        for(int j = 0; j < MatrixSize_y; j++){
////            floorMatrix[i][j] = 0;
//            if (i==j) floorMatrix[I2I(i,j)] = 1000;
//        }
//    }

}
/****************************************/
/****************************************/
void CArk::Reset() {
    m_messaging_counter = 0;
    m_floor_counter = 0;

    //Initialize the FloorMatrix
    memset(floorMatrix, 0, MatrixSize_x*MatrixSize_y*sizeof(float));

}
/****************************************/
/****************************************/

void CArk::write_file(){
	std::ofstream m_oLogFile;
	m_oLogFile.open(m_strOutputFilename.c_str(), std::ios::out | std::ios::trunc);
    	if(m_oLogFile.fail()) {
    		THROW_ARGOSEXCEPTION("Error opening file \"" << m_strOutputFilename << "\".");
    	}
    	for(UInt16 i = 0; i < foodVector.size(); ++i){
    		if (foodVector.at(i).id == 0) continue; // skip home
    		m_oLogFile << collected[foodVector.at(i).id] << "\t";
    	}

	for(UInt16 i = 0; i < foodVector.size(); ++i){
		if (foodVector.at(i).id == 0) continue; // skip home
		//fprintf (pFile, "Distribution in source %d: %d\n", i,connectTrial[i]);
		m_oLogFile << onPath.at(i) << "\t";
	}
	m_oLogFile << std::endl;
	m_oLogFile.close();

}

/****************************************/
/****************************************/
void CArk::Destroy() {
    
    //Write the file of experiments
    write_file();
    
    //Delete floor Matrix
    free(floorMatrix);

}


/****************************************/
//This function calculates the parameters of the evaporation and diffusion and actualize the matrix
/****************************************/
void CArk::updatePheromoneMatrix() {
//	for (int i =0; i< foodVector.size(); ++i){
//		if (foodVector.at(i).id==0) continue;
//		LOG << i << ":" << collected[foodVector.at(i).id] << " || ";
//	}LOG << endl;

    //Save the previous matrix in an static array
    float *aux_floorMatrix; //[MatrixSize_x][MatrixSize_y];
    aux_floorMatrix = (float*) malloc(MatrixSize_x*MatrixSize_y*sizeof(float));
    memcpy(aux_floorMatrix, floorMatrix, sizeof(float)*MatrixSize_x*MatrixSize_y );
    float diff_I, diff_J;
    float dt1=m_update_matrix_timesteps*0.1;
    //float dt2=0.1;
    float diffusionComponent = 0;
    
    //All except the borders
    for(int i = 1; i < MatrixSize_x-1; i++){
        for(int j = 1; j < MatrixSize_y-1; j++){
                //Calculate the vertical diffusion
                diff_I = aux_floorMatrix[I2I(i+1,j)]-2*aux_floorMatrix[I2I(i,j)]+aux_floorMatrix[I2I(i-1,j)];
                //Calculate the horizontal diffusion
                diff_J = aux_floorMatrix[I2I(i,j+1)]-2*aux_floorMatrix[I2I(i,j)]+aux_floorMatrix[I2I(i,j-1)];
            
                //Evaporation and diffusion equation
                diffusionComponent = diffusion_rate*(diff_I + diff_J)*dt1;
                if (diffusionComponent > maxPheroLimit) diffusionComponent = maxPheroLimit;
                floorMatrix[I2I(i,j)] =  diffusionComponent - (evaporation_rate*dt1 - 1.0)*aux_floorMatrix[I2I(i,j)];
                
                if(floorMatrix[I2I(i,j)] > maxPheroLimit) floorMatrix[I2I(i,j)] = maxPheroLimit;
                if(floorMatrix[I2I(i,j)] < 0.5)	floorMatrix[I2I(i,j)] = 0;
        }
    }
}


/****************************************/
/****************************************/
void CArk::PreStep(){
//	kb_positions.clear();
    //Actualize the floor every 10 steps
	m_floor_counter++;
    if(m_floor_counter >= m_update_matrix_timesteps){
        //Reduce the pheromon: Evaporation and diffusion
    		updatePheromoneMatrix();
        m_floor_counter = 0;
        GetSpace().GetFloorEntity().SetChanged();
    }
    // Send messages to the kilobots using ohc every 2 clock ticks (0.2s)
    m_messaging_counter++;
    if(m_messaging_counter >= 2){
        m_messaging_counter = 0;
        CKilobotCommunicationMedium& m_cMedium(m_cSimulator.GetMedium<CKilobotCommunicationMedium>("kilocomm"));
        CKilobotCommunicationMedium::TAdjacencyMatrix& m_tCommMatrix(m_cMedium.GetCommMatrix());
        
        std::fill(onPath.begin(), onPath.end(), 0);

        for(unsigned int i=0;i< m_KilobotsEntities.size();i++){

            Get_message_to_send(m_KilobotsEntities[i],m_messages[i]);
            
            if(m_tx_flag) {
                m_ohcCommunicationEntities[i].Enable();
                m_ohcCommunicationEntities[i].SetTxStatus(CKilobotCommunicationEntity::TX_ATTEMPT);
                m_ohcCommunicationEntities[i].SetTxMessage(&m_messages[i]);
                
                m_tCommMatrix[ (m_KilobotsEntities[i]->GetKilobotCommunicationEntity()).GetIndex() ].insert(&m_ohcCommunicationEntities[i]);
            }
            m_tx_flag=false;
        }
    }
}
/****************************************/
/****************************************/
void CArk::PostStep(){
    // do nothing
}
/****************************************/
/****************************************/
void CArk::CreateOhcCommunicationEntities(unsigned int number_of_comm_entities){
    
    m_ohcCommunicationEntities=KCVector(number_of_comm_entities, CKilobotCommunicationEntity(NULL,"kilocomm_0",9,1.0,m_cCommAnchor,m_ohcEmbodiedEntity));
    m_messages=KMVector(number_of_comm_entities);
}

/****************************************/
/****************************************/
void CArk::GetKilobotsEntities(){
    
    //Go through all the robots in the environment and create a vector of pointers on their entities
    CSpace::TMapPerType& kilobots_map = CSimulator::GetInstance().GetSpace().GetEntitiesByType("kilobot");
    
    for(CSpace::TMapPerType::iterator it = kilobots_map.begin(); it != kilobots_map.end();++it){
        m_KilobotsEntities.push_back(any_cast<CKilobotEntity*>(it->second));
    }
}

/****************************************/
/****************************************/
CVector2 CArk::GetKilobotPosition(CKilobotEntity* kilobot_entity){
	CVector3 pos3D = kilobot_entity->GetEmbodiedEntity().GetOriginAnchor().Position;
	return CVector2(pos3D.GetX(), pos3D.GetY());
}

CRadians CArk::GetKilobotOrientation(CKilobotEntity* kilobot_entity){
	CRadians cZAngle;
	CRadians cYAngle;
	CRadians cXAngle;
    CQuaternion cQuaternion = kilobot_entity->GetEmbodiedEntity().GetOriginAnchor().Orientation;
    cQuaternion.ToEulerAngles(cZAngle,cYAngle, cXAngle);
    return cZAngle;
}

/****************************************/
/****************************************/

void CArk::Get_message_to_send(CKilobotEntity* kilobot_entity,message_t& message){
	//Constant to know how small it has to be the circle of food when the ants take it
	//  const float reductFood = 0.005;

	//Get the kilobot position and orientation
	CVector2 kb_pos=GetKilobotPosition(kilobot_entity);
	CRadians orientation = GetKilobotOrientation(kilobot_entity);

	/****************************************/
	/****************************************/
	//Get positions and check if it is at home or food

	//Adapt the position to positive values and the position of the floor
//	track_x = kb_x + ArenaSize_x/2.0;
//	track_y = kb_y + ArenaSize_y/2.0;
//	track_x = round(track_x/(m_vSpaceSize.GetX()/MatrixSize_x));
//	track_y = round(track_y/(m_vSpaceSize.GetY()/MatrixSize_y));
	float track_x = kb_pos.GetX() - m_vSpaceLimits.GetX();
	float track_y = kb_pos.GetY() - m_vSpaceLimits.GetY();
	track_x = floor( track_x * m_iCellsPerMetre );
	track_y = floor( track_y * m_iCellsPerMetre );

	// store the home area location
	FoodClass homeArea;
	for (UInt16 i = 0; i < foodVector.size(); ++i){
		if (foodVector.at(i).id == 0){ // home area has ID==0
			homeArea.position = foodVector.at(i).position;
			homeArea.radius = foodVector.at(i).radius;
			break;
		}
	}

	// check which source of food have found
	UInt16 foodSourceIdx = 0;
	float m,c;
	float distance;
	bool atFood = false;
	for (UInt16 i = 0; i < foodVector.size(); ++i){
		if (foodVector.at(i).id == 0){ continue; }
		//Check if it is inside a food source
		Real distanceToFood = CVector2( foodVector.at(i).position - kb_pos ).Length();
		if (foodVector.at(i).radius != 0 && distanceToFood <= foodVector.at(i).radius){
			foodSourceIdx = i;
			atFood = true;
		}

		//Look if the kilobots are in the margins of any food

		//Segment between food and home y = mx + c
//		if (foodVector.at(i).position.GetX() - homeArea.position.GetX()!=0){
		m = (foodVector.at(i).position.GetY() - homeArea.position.GetY()) / (foodVector.at(i).position.GetX() - homeArea.position.GetX());
//		}
		c = homeArea.position.GetY() - m * homeArea.position.GetX();
		distance = abs(m*kb_pos.GetX() - kb_pos.GetY() + c)/sqrt(m*m + 1);

		if (distance < 0.2 && ((kb_pos.GetX() < homeArea.position.GetX() && kb_pos.GetX() > foodVector.at(i).position.GetX()) || (kb_pos.GetX() > homeArea.position.GetX() && kb_pos.GetX() < foodVector.at(i).position.GetX())) && ((kb_pos.GetY() < homeArea.position.GetY() && kb_pos.GetY() > foodVector.at(i).position.GetY()) || (kb_pos.GetY() > homeArea.position.GetY() && kb_pos.GetY() < foodVector.at(i).position.GetY()))){
			onPath.at(i) = onPath.at(i)+1;
		}
	}

	/****************************************/
	/****************************************/

	CVector2 vectorToHome = homeArea.position - kb_pos;

	//Look if it is in the food and it is not going home
	bool atHome = false;
	if (atFood && hasFood[GetKilobotId(kilobot_entity)] == 0){
		hasFood[GetKilobotId(kilobot_entity)] = foodVector.at(foodSourceIdx).id;
		//LOG << "Kilobot with id " << GetKilobotId(kilobot_entity) << " has collected food" << std::endl;
	}else{
		//Look if it is at home and is searching for home and add the food in case that it is
//		if ((fabs(kb_x - HOME_X) <= rad_SOURCE[0]) && (fabs(kb_y - HOME_Y) <= rad_SOURCE[0]) && GetKilobotLedColor(kilobot_entity) != CColor::RED){
		if ( vectorToHome.Length() <= homeArea.radius ){
			//LOG << "Kilobot with id " << GetKilobotId(kilobot_entity) << " has deposited food" << std::endl;
			//Find home, go food
			atHome = true;
			if (hasFood[GetKilobotId(kilobot_entity)] > 0) {
				collected[hasFood[GetKilobotId(kilobot_entity)]]++;
			}
			hasFood[GetKilobotId(kilobot_entity)] = 0;
		}
	}

	/****************************************/
	/****************************************/
	//Calculate the angle and the distance to home
	Real angleToHome = ToDegrees( vectorToHome.Rotate(-orientation).Angle() ).UnsignedNormalize().GetValue();

	/****************************************/
	/****************************************/
	//If it is going from Food to Home (led blue), print pheromone - update the floorMatrix

	if (GetKilobotLedColor(kilobot_entity) == CColor::BLUE){
		floorMatrix[I2I( track_x, track_y) ] += pheromone_amount;
	}

	/****************************************/
	/****************************************/
	//Calculate the orientation of the kilobots and save the floor values that the antenae can feel

	//If it is in the corners, don't substract the numbers
	int radDetection = 5;
	int phToSend[4] = {0,0,0,0};
	CRange<CDegrees> plusMinus90( CDegrees(-90), CDegrees(90));

	//Check if pheromone in their surrondings
	for(int i = max(0, (int)(track_x - radDetection)) ; i < min(MatrixSize_x, (int)(track_x + radDetection)); i++){
		for(int j = max(0, (int)(track_y - radDetection)) ; j < min(MatrixSize_y, (int)(track_y + radDetection)); j++){
			//If it is not him, not home and not food, analyse it
			if ( i == (int)track_x && j == (int)track_y ){ continue; }
			if(floorMatrix[I2I(i,j)] > 0){
//				CVector2 robot_pos(track_x, track_y);
//				CVector2 phero_pos(i, j);
//				CVector2 fromRobotToPhero = (phero_pos - robot_pos).Rotate(-orientation);

				// convert phero_pos to real world coordinates
				Real pt_x = Real(i) / m_iCellsPerMetre;
				Real pt_y = Real(j) / m_iCellsPerMetre;
				pt_x += m_vSpaceLimits.GetX() + 0.5/m_iCellsPerMetre; // adding half-cell-size
				pt_y += m_vSpaceLimits.GetY() + 0.5/m_iCellsPerMetre; // adding half-cell-size
				CVector2 phero_pos(pt_x, pt_y);

				CVector2 fromRobotToPhero = (phero_pos - kb_pos).Rotate(-orientation);

				//Compare with the orientation, don't use if it is not detected from the antenae
				if ( plusMinus90.WithinMinBoundIncludedMaxBoundIncluded( ToDegrees(fromRobotToPhero.Angle()).SignedNormalize() ) ){
					//if(GetKilobotLedColor(kilobot_entity) == CColor::GREEN)  printf("inside %f angleToHome %f \n", angleToKilobot, angleToHome);
					if (floor(ToDegrees(fromRobotToPhero.Angle()).UnsignedNormalize().GetValue()/45.0) == 6) phToSend[0] = 1;
					if (floor(ToDegrees(fromRobotToPhero.Angle()).UnsignedNormalize().GetValue()/45.0) == 7) phToSend[1] = 1;
					if (floor(ToDegrees(fromRobotToPhero.Angle()).UnsignedNormalize().GetValue()/45.0) == 0) phToSend[2] = 1;
					if (floor(ToDegrees(fromRobotToPhero.Angle()).UnsignedNormalize().GetValue()/45.0) == 1) phToSend[3] = 1;
				}
			}
		}
	}
	//    }
	//if(GetKilobotLedColor(kilobot_entity) == CColor::GREEN && !destHome && searchForFood != 1 && phToSend == 0 && phToSend[GetKilobotId(kilobot_entity)][1] == 5 && phToSend[GetKilobotId(kilobot_entity)][2] == 5 && phToSend[GetKilobotId(kilobot_entity)][3] == 5){
	//LOGERR << "Kilobot with id " << GetKilobotId(kilobot_entity) << " has lost the ph trail" << std::endl;
	// }


	/****************************************/
	/****************************************/
	//Construct the message to send to the kilobots

	uint8_t msg_type = 0;
	uint16_t msg_data = 0;
	message.type=NORMAL;
	message.data[0] = 0;
	message.data[1] = 0;
	message.data[2] = 0;

	//At food - Needs the quality instead of the pherozone
	if (atFood){
		//msg_data = 512;
		//Save the quality in MSB of data0
		//msg_data = (quality << 4) + msg_data;

		msg_data = foodVector.at(foodSourceIdx).quality;
		msg_data = (msg_data << 4);
		msg_data = msg_data + 512;
	}else{
		//Send the pheromone zones
		for(int i = 0; i < 4; i++){
			if (phToSend[i] != 0) {
				//For every number send 1,2,4 or 8 or the sum to know where are ph
				msg_data = pow(2,i) + msg_data;
			}
		}
		//Pherozones in the 4 MSB
		msg_data =  (msg_data << 4) ;
	}
	//At home
	if (atHome) msg_data = msg_data + 256;

	//Angle to home in the 4 LSB
	uint8_t ath = round(angleToHome/45.0);
	ath = (ath == 8)? 0 : ath;
	msg_type = ath;


	message.data[0] = message.data[0] | (GetKilobotId(kilobot_entity) >> 2);
	message.data[1] = message.data[1] | (GetKilobotId(kilobot_entity) << 6);
	message.data[1] = message.data[1] | (msg_type << 2);
	message.data[1] = message.data[1] | (msg_data >> 8);
	message.data[2] = message.data[2] | msg_data;

	// pack data with unused message at the end
	//message_t msg;
	uint16_t void_id = 1023;
	uint8_t void_type = 0;
	uint16_t void_data = 0;

	for (int i = 1; i < 3; ++i) {
		message.data[i*3] = message.data[i*3] | (void_id >> 2);
		message.data[1+i*3] = message.data[1+i*3] | (void_id << 6);
		message.data[1+i*3] = message.data[1+i*3] | (void_type << 2);
		message.data[1+i*3] = message.data[1+i*3] | (void_data >> 8);
		message.data[2+i*3] = message.data[2+i*3] | void_data;
	}

	message.crc =0;
	m_tx_flag=true;

//	if (atHome){
//		cout << "ID:" << kilobot_entity->GetId() << " position:" << GetKilobotPosition(kilobot_entity) << " dist-to-Home: " << vectorToHome.Length() << endl;
//		cout << "ID:" << kilobot_entity->GetId() << " dist to ("<<HOME_X << "," << HOME_Y << "):" << ( CVector2(kb_pos.GetX(),kb_pos.GetY()) - CVector2(0,0)  ).Length() << endl;
//	}
	//    if (kilobot_entity->GetId() == "kb5"){
	//    	cout << "At home:" << atHome << " atFood:" << atFood << " angle-to-Home: (" << angleToHome << " | " << round(angleToHome/45) << ") phToSend:[" << phToSend[0] << "," << phToSend[1] << "," << phToSend[2] << "," << phToSend[3] << "]" << endl;
	//    }
//	LOG << "Kilobot with id " << GetKilobotId(kilobot_entity) << " at distance " << vectorToHome.Length() << std::endl;
//	if (vectorToHome.Length() > 0.51) {
//		LOG << "phero:" << phToSend[0] <<phToSend[1] <<phToSend[2] <<phToSend[3] <<" AtH:" << angleToHome << " (" << ath << ")"<< endl;
//	}
}

/****************************************/
/****************************************/
UInt16 CArk::GetKilobotId(CKilobotEntity *kilobot_entity){
    std::string entity_id((kilobot_entity)->GetControllableEntity().GetController().GetId());
    return std::stoul(entity_id.substr(2));
}
/****************************************/
/****************************************/

CColor CArk::GetKilobotLedColor(CKilobotEntity *kilobot_entity){
    return kilobot_entity->GetLEDEquippedEntity().GetLED(0).GetColor();
}

/****************************************/
/****************************************/
CColor CArk::GetFloorColor(const CVector2 &c_position_on_plane) {

	double pt_x(c_position_on_plane.GetX()),pt_y(c_position_on_plane.GetY());
	CColor color=CColor::WHITE;
	for (UInt16 i = 0; i < foodVector.size(); ++i){
		CVector2 vectorToArea = foodVector.at(i).position - c_position_on_plane;
		if( vectorToArea.Length() <= foodVector.at(i).radius ){
			//Color depending on the argos file
			color=inttoccolor(foodVector.at(i).colour);
			//If food look the quality and put the right color
			if(foodVector.at(i).colour == 9){
				color = CColor(round(24.5*(10-foodVector.at(i).quality/1.8)),10,5);
			}
		}
	}

    // Paint the pheromone
//    pt_x += ArenaSize_x/2;
//    pt_y += ArenaSize_y/2;
    pt_x -= m_vSpaceLimits.GetX();
    pt_y -= m_vSpaceLimits.GetY();
    float track_x = floor(pt_x* m_iCellsPerMetre);
    float track_y = floor(pt_y* m_iCellsPerMetre);

    //cout << " For pos " << c_position_on_plane << " cells are " << track_x << "," << track_y << endl;
	if (color != CColor::GREEN){
		if(floorMatrix[I2I(track_x,track_y)] >= 1){
			int pheroColourLimit = 1000;
			if (floorMatrix[I2I(track_x,track_y)] > pheroColourLimit){
				color= CColor::BLACK;
			} else {
				int normVal = floorMatrix[I2I(track_x,track_y)]*255/pheroColourLimit;
				color=CColor(230-normVal, 240, 255-normVal*0.7);
			}
		}
	}
    
    return color;
}

/****************************************/
/****************************************/

void CArk::GetOptions(TConfigurationNode& t_tree){
    
    TConfigurationNodeIterator itOptions;
    
    for(itOptions=itOptions.begin(&t_tree);
        itOptions!=itOptions.end();
        ++itOptions){
        if(itOptions->Value()=="option") AddOption(*itOptions);
        
    }
    
}

/****************************************/
/****************************************/

void CArk::AddOption(TConfigurationNode& t_node){
	FoodClass newFood;
    GetNodeAttribute(t_node, "id", newFood.id);
    if (newFood.id < 0) return;
    GetNodeAttribute(t_node, "quality", newFood.quality);
    GetNodeAttribute(t_node, "position", newFood.position);
    GetNodeAttribute(t_node, "radius", newFood.radius);
    GetNodeAttribute(t_node, "color", newFood.colour);
    
    foodVector.push_back(newFood);
}

/****************************************/
/****************************************/

CColor CArk::inttoccolor(int color_id){
    switch (color_id)
    {
        case 0:
            return CColor::BLACK;
            break;
            
        case 1:
            return CColor::WHITE;
            break;
            
        case 2:
            return CColor::RED;
            break;
            
        case 3:
            return CColor::GREEN;
            break;
            
        case 4:
            return CColor::BLUE;
            break;
            
        case 5:
            return CColor::MAGENTA;
            break;
            
        case 6:
            return CColor::CYAN;
            break;
            
        case 7:
            return CColor::YELLOW;
            break;
            
        case 8:
            return CColor::ORANGE;
            break;
            
        case 9:
            return CColor::BROWN;
            break;
            
        case 10:
            return CColor::PURPLE;
            break;
            
        default:
            return CColor::WHITE;
            break;
    }
}



REGISTER_LOOP_FUNCTIONS(CArk, "ark_loop_functions2")
