/*
 * MessageCreator.cpp
 *
 *  Created on: Feb 4, 2016
 *      Author: Mathias Neerup
 */

#include "MessageCreator.h"

MessageCreator::MessageCreator() {
	// TODO Auto-generated constructor stub
}

canMSG MessageCreator::Create_ReqAddr(int type, int canId){
	canMSG msg_out;

	msg_out.id = CAN_FID_REQ_ADDR | CAN_EFF;

	// ?? EF, CD, AB, 89 is uuid in AutoQuad ??
	std::vector<int> v {'\x01','\xEF','\xCD',mySession.uuid,type, canId,'\x23','\x01'}; // '\x23','\x01'

	for(int i = 0; i < v.size(); ++i){
		msg_out.data[i] = v[i];
	}
	msg_out.length = v.size();

	return msg_out;
}

canMSG MessageCreator::Create_Stream_VEL(int16_t velN, int16_t velE, int16_t velD, int16_t speed, unsigned int doc) {

	canMSG msg_out;
	msg_out.id = CAN_FID_TELEM | CAN_EFF | (mySession.source_id << (14-3)) | (doc << (19-3));

	msg_out.data[0] = (velN & 0x00FF);
	msg_out.data[1] = ((velN & 0xFF00) >> 8);

	msg_out.data[2] = (velE & 0x00FF);
	msg_out.data[3] = ((velE & 0xFF00) >> 8);

	msg_out.data[4] = (velD & 0x00FF);
	msg_out.data[5] = ((velD & 0xFF00) >> 8);

	msg_out.data[6] = (speed & 0x00FF);
	msg_out.data[7] = ((speed & 0xFF00) >> 8);

	msg_out.length = 8;

	return msg_out;
}
canMSG MessageCreator::Create_Stream_ACC(uint8_t satellites, uint8_t fix, uint8_t sAcc, uint8_t cAcc, uint8_t hAcc, uint8_t vAcc, int16_t heading, unsigned int doc) {

	canMSG msg_out;
	msg_out.id = CAN_FID_TELEM | CAN_EFF | (mySession.source_id << (14-3)) | (doc << (19-3));

	msg_out.data[0] = satellites;
	msg_out.data[1] = fix;
	msg_out.data[2] = sAcc;
	msg_out.data[3] = cAcc;
	msg_out.data[4] = hAcc;
	msg_out.data[5] = vAcc;
	msg_out.data[6] = (heading & 0x00FF);
	msg_out.data[7] = ((heading & 0xFF00) >> 8);

	msg_out.length = 8;

	return msg_out;
}
canMSG MessageCreator::Create_Stream_DOP(int pDOP, int hDOP, int vDOP, int tDOP, int nDOP, int eDOP, int gDOP, unsigned int doc) {

	canMSG msg_out;
	msg_out.id = CAN_FID_TELEM | CAN_EFF | (mySession.source_id << (14-3)) | (doc << (19-3));

	msg_out.data[0] = pDOP;
	msg_out.data[1] = hDOP;
	msg_out.data[2] = vDOP;
	msg_out.data[3] = tDOP;
	msg_out.data[4] = nDOP;
	msg_out.data[5] = eDOP;
	msg_out.data[6] = gDOP;

	msg_out.length = 7;

	return msg_out;
}
canMSG MessageCreator::Create_Stream(double value, unsigned int doc) {

	canMSG msg_out;
	msg_out.id = CAN_FID_TELEM | CAN_EFF | (mySession.source_id << (14-3)) | (doc << (19-3));

	const unsigned char * value_double = reinterpret_cast<const unsigned char*>(&value);

	for(int i = 0; i < sizeof(double); ++i){
		msg_out.data[i] = value_double[i];
	}
	msg_out.length = sizeof(double);

	return msg_out;
}

canMSG MessageCreator::Create_SendACK(){
	canMSG msg_out;

	msg_out.id = CAN_FID_ACK | CAN_EFF | mySession.sequence_id;
	msg_out.length = 0;

	return msg_out;
}
