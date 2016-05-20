/*
 MMN - Mathias Mikkel Neerup manee12@student.sdu.dk
 */

#include <sstream>
#include <iostream>

#include <stdint.h>
#include "AutoQuad.h"
#include "Session.h"
#include "gps_conv.h"
#include "msgs/nmea.h"
#include <cmath>
#include <sstream>



#define CAN_DOC_LAT 0x01
#define CAN_DOC_LON 0x02
#define CAN_DOC_DOP 0x03
#define CAN_DOC_ACC 0x04
#define CAN_DOC_VEL 0x05
#define CAN_DOC_ALT 0x06


#define GPRMC_SPEED_KNOTS  6
#define GPRMC_TRUE_COURSE  7


#define deg2rad M_PI/180.0
//#define LATITUDE_DOC   0x01
//#define LONGITUDE_DOC  0x02
//#define HDOP_DOC       0x03
//#define FIX_DOC        0x04
//#define satellites_DOC 0x05
//#define ALTITUDE_DOC   0x06


#define MSG_RESET       1
#define MSG_OK_ADDR     2
#define MSG_TELEM_VALUE 3
#define MSG_TELEM_RATE  4

#define ST_ACKNOWLEDGED 1


class RegisterNode_node : public AutoQuad {
private:
	MessageCreator messageCreator;
	int last_msg;
	int state = 0;

	ros::Subscriber nmea_sub;
	msgs::nmea last_gps_msg;
	std::string topic_nmea_from_gps_sub;


	// DOP
	uint8_t pDOP = 4;
	uint8_t hDOP = 4;
	uint8_t vDOP = 4;
	uint8_t tDOP = 4;
	uint8_t nDOP = 4;
	uint8_t eDOP = 4;
	uint8_t gDOP = 4;


	// Velocities
	int16_t velN  = 0;
	int16_t velE  = 0;
	int16_t velD  = 0;
	int16_t speed = 0;


	// Accuracy
	uint8_t sAcc = 0;
	uint8_t cAcc = 0.3;
	uint8_t hAcc = 20; // divide by 10 on drone
	uint8_t vAcc = 30; // divide by 10 on drone
	int16_t heading = 0;

	double last_altitude = 0;
public:
	RegisterNode_node(int argc, char** argv): AutoQuad(argc, argv){
		// UUID of this CAN node
		messageCreator.mySession.uuid = 0xAB;
		ros::NodeHandle nh("~");
		nh.param<std::string> ("nmea_from_device_sub", topic_nmea_from_gps_sub, "/fmData/nmea_from_gps");
		nmea_sub = nh.subscribe(topic_nmea_from_gps_sub, 1000, &RegisterNode_node::nmea_callback, this);
	}

	void nmea_callback(const msgs::nmea::ConstPtr& nmea_msg){
		if(!nmea_msg->type.compare("GPRMC") && nmea_msg->valid == true) {
			ROS_WARN("Recv. GPRMC");
			double speed_knots = atof(nmea_msg->data[GPRMC_SPEED_KNOTS].c_str());
			double true_course = atof(nmea_msg->data[GPRMC_TRUE_COURSE].c_str());


			// Convert ground velocity knots to m/s
			double vel_g = speed_knots * 1852.0/3600.0;

			// Convert from compass course (cw degrees origin y-axis)
			// to unit circle radians (ccw origin x-axis)
			double theta = (90.0 - true_course)*deg2rad;


//			ROS_WARN("Theta: %f", theta);
			if(theta > 2*M_PI || theta < 0){
				std::cout << "Theta: " << theta << std::endl;
				ROS_ERROR("ERROR------------------------------------------------");
			}

			velE  = (int16_t)(cos(theta)*vel_g*100); // Easting component of velocity
			velN  = (int16_t)(sin(theta)*vel_g*100); // Northing component of velocity
			speed = (int16_t)(vel_g*100); // scale up, scale down on drone
			heading = (int16_t)(theta*100);

		} else if(!nmea_msg->type.compare("GPGGA") && nmea_msg->valid == true) {

			double latitude = 0;
			double longitude = 0;
                        double altitude = 0;
			double hdop = 0;
			double fix = 0;
			double satellites = 0;

			if(nmea_msg->data[1] != ""){
				latitude =  nmea_latlon( (char* ) nmea_msg->data[1].c_str() );
			}

			if(nmea_msg->data[3] != ""){
				 longitude = nmea_latlon( (char* ) nmea_msg->data[3].c_str() );
			}

			if(nmea_msg->data[8] != ""){
	                         altitude = atof( nmea_msg->data[8].c_str() );
			}

			if(nmea_msg->data[7] != ""){
				 hdop = atof( nmea_msg->data[7].c_str() );
			}

			if(nmea_msg->data[5] != ""){
				 fix = atof(nmea_msg->data[5].c_str() );
			}

			if(nmea_msg->data[6] != ""){
				 satellites = atof( nmea_msg->data[6].c_str() );
			}


                        ROS_INFO("POS in lat: %f long: %f altitude: %f ", latitude ,longitude, altitude);

			velD = -(int16_t)( ( (altitude-last_altitude)/1 ) * 100); // dt = 1
			last_altitude = altitude;

			if(state == ST_ACKNOWLEDGED){
				ROS_INFO("POS out");

				// Latitude
				canMSG canMessage = messageCreator.Create_Stream(latitude, CAN_DOC_LAT);
				pub_recv.publish(canMessage);

				// Longitude
				canMessage = messageCreator.Create_Stream(longitude, CAN_DOC_LON);
				pub_recv.publish(canMessage);


				// multiplied by 10 in all DOP
				pDOP = 1.75*hdop*10;
				hDOP = 1*hdop*10;
				vDOP = 1.5*hdop*10;
				tDOP = 1.5*hdop*10;
				nDOP = 0.7*hdop*10;
				eDOP = 0.7*hdop*10;
				gDOP = hdop*10;


				canMessage = messageCreator.Create_Stream_DOP(pDOP, hDOP, vDOP, tDOP, nDOP, eDOP, gDOP, CAN_DOC_DOP);
				pub_recv.publish(canMessage);


				canMessage = messageCreator.Create_Stream_VEL(velN, velE, velD, speed, CAN_DOC_VEL);
				pub_recv.publish(canMessage);



				canMessage = messageCreator.Create_Stream_ACC(satellites, fix, sAcc, cAcc, hAcc, vAcc, heading, CAN_DOC_ACC);
				pub_recv.publish(canMessage);

//				canMessage = messageCreator.Create_Stream(hdop, HDOP_DOC);
//				pub_recv.publish(canMessage);

				// FIX
//				canMessage = messageCreator.Create_Stream(fix, FIX_DOC);
//				pub_recv.publish(canMessage);

				// satellites
//				canMessage = messageCreator.Create_Stream(satellites, satellites_DOC);
//				pub_recv.publish(canMessage);

				// Altitude
				canMessage = messageCreator.Create_Stream(altitude, CAN_DOC_ALT);
				pub_recv.publish(canMessage);
			} else {
				ROS_WARN("Node not acknowledged");
			}
		 }
	}

	void onTimer(const ros::TimerEvent& event){
	}

	void spin(){
		ros::spin();
	}

	bool packet_to_me(int id = 0){
		int retval = false;
		// data packet: 1 EF CD AB 0 0 0 0   AB is uuid sent by this node
		int target_id = ((id & CAN_TID_MASK) >> (9-3));
		if( latest_packet.data[3] == messageCreator.mySession.getUuid() || messageCreator.mySession.source_id == target_id){
			retval = true;
		}
		return retval;
	}

	void recv_reset_msg() {
		ROS_WARN("Reset msg received");
		canMSG canMessage = messageCreator.Create_ReqAddr(CAN_TYPE_SENSOR, CAN_SENSORS_GPS_LAT);
		pub_recv.publish(canMessage);
		last_msg = MSG_RESET;
	}

	void recv_ok_addr_msg(int id) {
		if( packet_to_me() ){
			if(last_msg == MSG_RESET){
				ROS_INFO("Node got ack");
				messageCreator.mySession.updateSession(id);
				last_msg = MSG_OK_ADDR;
			}
		}
	}

	void recv_telem_value_msg(int id){
		if (packet_to_me(id)){
			if(last_msg == MSG_OK_ADDR){
				ROS_WARN("TELEM_VALUE");
				canMSG canMessage = messageCreator.Create_SendACK();
				pub_recv.publish(canMessage);
				last_msg = MSG_TELEM_VALUE;
			}
		}
	}
	void recv_telem_rate_msg(int id){
		if (packet_to_me(id)){
			if(last_msg == MSG_TELEM_VALUE){
				ROS_WARN("TELEM_RATE");
				canMSG canMessage = messageCreator.Create_SendACK();
				pub_recv.publish(canMessage);
				last_msg = MSG_TELEM_RATE;
				state = ST_ACKNOWLEDGED;
			}
		}
	}
};
int main(int argc, char **argv){
	AutoQuad* autoquad = new RegisterNode_node(argc, argv);


	autoquad->spin();
	return 0;
}
