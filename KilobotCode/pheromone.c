#include "kilolib.h"
#include <stdlib.h>
//#include <stdio.h>
#include <math.h>
//#define DEBUG
//#include "debug.h"

#define SEED_ID 0
#define STRAIGHT_TIME 240
#define TURN_RAND_TIME 80
#define MIN_TIME_BEFORE_SPIN 20
#define DEPOSIT_PHEROMONE_STEP 64
// in argos 0.44 on real robots 1.07
//#define ticks_per_rotation_degree 0.44
#define ticks_per_rotation_degree 1.07

/* Enum for boolean flags */
typedef enum {
	false = 0,
	true = 1,
} bool;

/* Enum for different motion types */
typedef enum {
	STOP = 0,
	FORWARD,
	TURN_LEFT,
	TURN_RIGHT,
} motion_t;

/* Enum for different motion types */
typedef enum {
	SEARCH_FOOD = 0,
	GO_HOME,
	FOLLOW_PH,
	TURN_KILOBOT,
} behave_t;

behave_t behave;

motion_t current_motion;
uint32_t last_motion_update = 0;
uint32_t timesToTurn = 0;
bool turn180completed = false;

//bool new_message;
bool updated_home_angle;
bool atHome;
bool atFood;
bool atPheromone;
uint32_t last_motion_ticks;
uint32_t last_pheromone_deposit_ticks;

int angleToHome; // received as round( a/45 ) and computed in value between 0 and 360
float best_pheromone_angle = 0;
uint32_t countLost = 0;
int foodQuality;

message_t message;

bool runtime_identification = false; // when true the runtime identification is ongoing
uint8_t backup_LED = 0; // variable to store LED colour before runtime identification (0 - OFF, 1 - RED, 2 - GREEN, 3 - BLUE)

// Function to handle motion.
void set_motion(motion_t new_motion){
    // Only take an an action if the motion is being changed.
    if (current_motion != new_motion){
        current_motion = new_motion;
        
        if (current_motion == STOP){
            set_motors(0, 0);
        }
        else if (current_motion == FORWARD){
            spinup_motors();
            set_motors(kilo_straight_left, kilo_straight_right);
        }
        else if (current_motion == TURN_LEFT){
            spinup_motors();
            set_motors(kilo_turn_left, 0);
        }
        else if (current_motion == TURN_RIGHT){
            spinup_motors();
            set_motors(0, kilo_turn_right);
        }
    }
}

void setup(){
    //Initialized the empty data
    srand(rand_hard());
	//rand_seed(rand_hard());
    last_motion_ticks = rand() % STRAIGHT_TIME;
//    printf("FOR DEBUG, reset last_motion_ticks\n");
//    srand(123456);
//    last_motion_ticks = 0;
    //behave = GO_HOME;
    behave = SEARCH_FOOD;
    atHome = false;
    atFood = false;
    atPheromone = false;
    angleToHome = 0;
    current_motion = FORWARD;
    updated_home_angle = false;
    foodQuality = 0;
}


//Decide a random motion
void set_random_motion(){
   switch( current_motion ) {
   case TURN_LEFT:
   case TURN_RIGHT:
	   if( kilo_ticks > last_motion_ticks + timesToTurn ) {
		   /* start moving forward */
		   last_motion_ticks = kilo_ticks;
		   set_motion(FORWARD);
//		   if (kilo_uid == 8) { printf("Go straight (%d): %d \n", kilo_ticks, timesToTurn); }
	   }
	   break;
   case FORWARD:
	   if( kilo_ticks > last_motion_ticks + STRAIGHT_TIME ) {
		   /* perform a radnom turn */
//		   if (kilo_uid == 8) { printf("Turn (%d): %d \n", kilo_ticks, timesToTurn); }
		   last_motion_ticks = kilo_ticks;
		   if( rand()%2 ) {
			   set_motion(TURN_LEFT);
		   }
		   else {
			   set_motion(TURN_RIGHT);
		   }
		   timesToTurn = (rand()%TURN_RAND_TIME) + 1;
	   }
	   break;
   case STOP:
   default:
	   set_motion(STOP);
   }
}


void checkPheroQuality(){
    // Probability of depositing pheromone
    int probability = foodQuality*10;

    //Random number between 0 and 99
    uint8_t randNum = rand() % 100;
    
    //If probability bigger, print ph
    if(probability > randNum){
        if (!runtime_identification) set_color(RGB(0, 0, 3));
	backup_LED = 3; // (0 - OFF, 1 - RED, 2 - GREEN, 3 - BLUE)
    }else{
        //Else, no pheromone, led off
        if (!runtime_identification) set_color(RGB(0, 0, 0));
	backup_LED = 0; // (0 - OFF, 1 - RED, 2 - GREEN, 3 - BLUE)
    }
}


void returnHome(){
	//Look the probability and change RGB in order to not put ph
	if(kilo_ticks > last_pheromone_deposit_ticks + DEPOSIT_PHEROMONE_STEP){
		checkPheroQuality();
		last_pheromone_deposit_ticks = kilo_ticks;
	}

	/* set motion depending on the received message */
	// If turning, I ignore messages
	// Turning time depends on the angle to home
	if(current_motion != FORWARD && kilo_ticks > last_motion_ticks + timesToTurn){
		timesToTurn = 0;
		set_motion(FORWARD);
	} else {
		//If new message, use the information to move
		if (updated_home_angle && kilo_ticks > last_motion_ticks + MIN_TIME_BEFORE_SPIN){
			updated_home_angle = false;
			// If the home is in front of the robot (with a margin of +-30 deg), it goes straight
			if ( angleToHome < 30 || angleToHome > 330 ){
				set_motion(FORWARD);
				//printf("[%d][%d] move FWD ", kilo_uid, kilo_ticks);
			} // Set the direction to turn left or right
			// If the angle is smaller than 180 degrees is faster to go left
			else if (angleToHome == 180){
				if (rand()%2) set_motion(TURN_LEFT); else set_motion(TURN_RIGHT);
			}else if (angleToHome < 180){
				set_motion(TURN_LEFT);
				//printf("[%d][%d] move LEFT ", kilo_uid, kilo_ticks);
			}else{
				//If the angle is bigger than 180 degrees is faster to go right
				angleToHome = 360 - angleToHome;
				set_motion(TURN_RIGHT);
				//printf("[%d][%d] move RIGHT ", kilo_uid, kilo_ticks);
			}
			timesToTurn = (uint32_t)(angleToHome * ticks_per_rotation_degree);
			//printf("-- TtT: %d \n", timesToTurn);
			last_motion_ticks = kilo_ticks;
		}
	}
}

float min(float a, float b){
	if (a < b) return a;
	else return b;
}

bool almostEq(float a, float b){
	if( fabs(a-b) < 0.001) return true;
	else return false;
}

void message_rx(message_t *m, distance_measurement_t *d){
	bool new_message = false;
	int sa_payload = 0;
	uint8_t sa_type = 0;

	if (m->type == 0) {
		// unpack message
		int id1 = m->data[0] << 2 | (m->data[1] >> 6);
		int id2 = m->data[3] << 2 | (m->data[4] >> 6);
		int id3 = m->data[6] << 2 | (m->data[7] >> 6);
		if (id1 == kilo_uid) {
			// unpack type
			sa_type = m->data[1] >> 2 & 0x0F;
			// unpack payload
			sa_payload = ((m->data[1]&0b11) << 8) | (m->data[2]);
			new_message = true;
		}
		if (id2 == kilo_uid) {
			// unpack type
			sa_type = m->data[4] >> 2 & 0x0F;
			// unpack payload
			sa_payload = ((m->data[4]&0b11)  << 8) | (m->data[5]);
			new_message = true;
		}
		if (id3 == kilo_uid) {
			// unpack type
			sa_type = m->data[7] >> 2 & 0x0F;
			// unpack payload
			sa_payload = ((m->data[7]&0b11)  << 8) | (m->data[8]);
			new_message = true;
		}
	} else if (m->type == 120) { // initial identification
		int id = (m->data[0] << 8) | m->data[1];
		if (id == kilo_uid) {
			set_color(RGB(0,0,3));
		} else {
			set_color(RGB(3,0,0));
		}
	} else if (m->type == 119) { // runtime identification
		int id = (m->data[0] << 8) | m->data[1];
		if (id >= 0){ // runtime identification ongoing
			runtime_identification = true;			
			if (id == kilo_uid) {
				set_color(RGB(0,0,3));
			} else {
				set_color(RGB(3,0,0));
			}
		} else { // runtime identification ended
			runtime_identification = false;
			// restore the LED colour (0 - OFF, 1 - RED, 2 - GREEN, 3 - BLUE)
			if (backup_LED == 0)
				set_color(RGB(0,0,0));
			else if (backup_LED == 1)
				set_color(RGB(3,0,0));
			else if (backup_LED == 2)
				set_color(RGB(0,3,0));
			else if (backup_LED == 0)
				set_color(RGB(0,0,3));
		}
	}

	if (new_message){
		float MaxDistToHome = 0;
		int angleRx = 0;
		float aux_phAngle = 0;
		// Storing the received angle to home
		// angleToHome = 0b0000001111 & sa_payload;
		angleToHome = sa_type;
		angleToHome = angleToHome*45;
		//printf("[%d][%d] Angle to home is %f \n", kilo_uid, kilo_ticks, angleToHome);
		updated_home_angle = true;

		// if (kilo_uid == 5){
		//	printf("[%d] angle-to-home (%f | %f) , type (%d)\n", kilo_ticks/3, tmp, angleToHome, sa_type);
		// }
		// int angle_int = (int)round(angleToHome);
		// printf("[%lu] (id:%hu) angle-to-home: ( %d | %d )\n", kilo_ticks, kilo_uid, angle_int, sa_type);

		// Storing information about pheromone
		atPheromone = false;
		best_pheromone_angle = -1;

		//Check if it is at food
		atFood = false;
		atHome = false;
		if (sa_payload >= 512){
			sa_payload = sa_payload - 512;
			//Save the quality of the food source
			foodQuality = sa_payload >> 4;
			atFood = true;
		}else{
			//If not at food, maybe at home
			if (sa_payload >= 256){
				atHome = true;
				sa_payload = sa_payload - 256;
			}

			//If not at food, save pherozones
			angleRx = (sa_payload >> 4) & 0b0000001111;

			//printf("[%d][%d] Phero is ", kilo_uid, kilo_ticks);
			//Find the angle farther from home
			if (angleRx >= 8){
				atPheromone = true;
				aux_phAngle = 67.5;
				angleRx = angleRx - 8;
				float diff = min(360 - fabs(aux_phAngle - angleToHome), fabs(aux_phAngle - angleToHome));
				if( diff > MaxDistToHome){
					MaxDistToHome = diff;
					best_pheromone_angle = aux_phAngle;
				}
				//printf(" 1 |");
			}// else {printf(" 0 |");}
			if (angleRx >= 4){
				atPheromone = true;
				aux_phAngle = 22.5;
				angleRx = angleRx - 4;
				float diff = min(360 - fabs(aux_phAngle - angleToHome), fabs(aux_phAngle - angleToHome));
				if ( almostEq(diff,MaxDistToHome) ){
					if (rand()%2) best_pheromone_angle = aux_phAngle;
					//printf(" eq ");
				} else if( diff > MaxDistToHome){
					MaxDistToHome = diff;
					best_pheromone_angle = aux_phAngle;
					//printf(" b ");
				}
				//printf(" 1 |");
			}// else {printf(" 0 |");}
			if (angleRx >= 2){
				atPheromone = true;
				aux_phAngle = 360-22.5;
				angleRx = angleRx - 2;
				float diff = min(360 - fabs(aux_phAngle - angleToHome), fabs(aux_phAngle - angleToHome));
				if ( almostEq(diff,MaxDistToHome) ){
					//if (kilo_ticks/3 == 280) { printf("RAND:%d\n",rand()); }
					if (rand()%2) best_pheromone_angle = aux_phAngle;
					//printf(" eq ");
				} else if( diff > MaxDistToHome){
					MaxDistToHome = diff;
					best_pheromone_angle = aux_phAngle;
					//printf(" b ");
				}
				//printf(" 1 |");
			}// else {printf(" 0 |");}
			if (angleRx >= 1){
				atPheromone = true;
				aux_phAngle = 360-67.5;
				angleRx = angleRx - 1;
				float diff = min(360 - fabs(aux_phAngle - angleToHome), fabs(aux_phAngle - angleToHome));
				if ( almostEq(diff,MaxDistToHome) ){
					if (rand()%2) best_pheromone_angle = aux_phAngle;
					//printf(" eq ");
				} else if( diff > MaxDistToHome){
					MaxDistToHome = diff;
					best_pheromone_angle = aux_phAngle;
					//printf(" b ");
				}
				//printf(" 1 |\n");
			}// else {printf(" 0 |\n");}
		}

//		if (kilo_uid == 5){
//		printf("RAND:%d\n",rand());
//		if (kilo_ticks/3 > 200){
//			printf("[%d][%d] At home (%d) , at Food (%d) , angle-to-home (%f) , best_pheromone_angle (%f)\n", kilo_uid, kilo_ticks/3, atHome, atFood, angleToHome, best_pheromone_angle);
//		}
	}
}

void loop(){
//	printf("[%d][%d] current state (%d)\n", kilo_uid, kilo_ticks, behave);
	switch (behave){
	case SEARCH_FOOD: {
		set_random_motion();
		if (atFood){ // if food found
			atFood = false;
			//Go at home leaving pheromone
			checkPheroQuality();
			last_pheromone_deposit_ticks = kilo_ticks;
			behave = GO_HOME;
			set_motion(STOP);
		} else if(atPheromone){ //Check if pheromone found
			if (!runtime_identification) set_color(RGB(3, 0, 0));
			backup_LED = 1; // (0 - OFF, 1 - RED, 2 - GREEN, 3 - BLUE)
			behave = FOLLOW_PH;
			last_motion_ticks = kilo_ticks - MIN_TIME_BEFORE_SPIN - 1;
		} else { // Green, random movement, searching food
			if (!runtime_identification) set_color(RGB(0, 3, 0));
			backup_LED = 2; // (0 - OFF, 1 - RED, 2 - GREEN, 3 - BLUE)
		}
		break;
	}
	case GO_HOME: {
		if(!atHome){
			//If there is food available go to collect it
			returnHome();
		}else{
			//At Home
			atHome = false;
			if (!runtime_identification) set_color(RGB(3, 0, 0));
			backup_LED = 1; // (0 - OFF, 1 - RED, 2 - GREEN, 3 - BLUE)
			//Turn 180 degrees to follow the PH
			behave = TURN_KILOBOT;
			timesToTurn = (uint32_t)(180 * ticks_per_rotation_degree);
			last_motion_ticks = kilo_ticks;
			turn180completed = false;
			//set_motion(TURN_LEFT);
			if (rand()%2) set_motion(TURN_LEFT); else set_motion(TURN_RIGHT);
		}
		break;
	}
	case TURN_KILOBOT: {
		if(atFood){
			atFood = false;
			checkPheroQuality();
			last_pheromone_deposit_ticks = kilo_ticks;
			behave = GO_HOME;
			set_motion(STOP);
		} else {
			if(kilo_ticks > last_motion_ticks + timesToTurn){
				//After turning 180 go forward to find the path
				set_motion(FORWARD);
				last_motion_ticks = kilo_ticks;
				timesToTurn = 15;
				if (turn180completed){
					behave = SEARCH_FOOD;
				} else {
					turn180completed = true;
				}
			}
		}
		//If it was lost but now it finds ph, check where to go
		if (atPheromone){
			behave = FOLLOW_PH;
			turn180completed = false;
			last_motion_ticks = kilo_ticks - MIN_TIME_BEFORE_SPIN - 1;
		}
		break;
	}
	case FOLLOW_PH: {
		if(atFood){
			atFood = false;
			checkPheroQuality();
			last_pheromone_deposit_ticks = kilo_ticks;
			behave = GO_HOME;
			set_motion(STOP);
		} else {
			// If pheromone is detected
			if(atPheromone && kilo_ticks > last_motion_ticks + MIN_TIME_BEFORE_SPIN){
				atPheromone = false;
				//If the angle is smaller than 180 degrees is faster to go left
				if (best_pheromone_angle < 180){
					//printf("[%d][%d] move (ph) LEFT ", kilo_uid, kilo_ticks);
					set_motion(TURN_LEFT);
				} else {
					//If the angle is bigger than 180 degrees is faster to go right
					best_pheromone_angle = 360 - best_pheromone_angle;
					//printf("[%d][%d] move (ph) RIGHT ", kilo_uid, kilo_ticks);
					set_motion(TURN_RIGHT);
				}
				timesToTurn = (uint32_t)(best_pheromone_angle * ticks_per_rotation_degree);
				//printf("-- TtT: %d \n", timesToTurn);
				last_motion_ticks = kilo_ticks;
				countLost = kilo_ticks;
			} else { // if I didn't receive atPheromone message

				// I keep turning and when I finished I try to move in the direction opposite to home
				if (kilo_ticks > last_motion_ticks + timesToTurn) {
					//Turn 180 degrees to follow the PH
					behave = TURN_KILOBOT;
					turn180completed = false;
					timesToTurn = (uint32_t)(180 * ticks_per_rotation_degree);

					last_motion_ticks = kilo_ticks;
					// select opposite direction than before
					if (current_motion == TURN_LEFT)
						set_motion(TURN_RIGHT);
					else
						set_motion(TURN_LEFT);
					// select random rotation direction
					//if( rand()%2 ) { set_motion(TURN_LEFT); }
					//else { set_motion(TURN_RIGHT); }
					//printf("[%d][%d] I might never enter here -- TtT: %d \n", kilo_uid, kilo_ticks, timesToTurn);
				}

				// if I don't receive information from ARK about Pheromone for more than 5 seconds, I declare myself as Lost and I start random walk! :-(
				if (kilo_ticks > countLost + 160) {
					behave = SEARCH_FOOD;
				}
			}
		}
		break;
	}
	}
}

int main(){
    kilo_init();
    kilo_message_rx = message_rx;
    kilo_start(setup, loop);
    
    return 0;
}
