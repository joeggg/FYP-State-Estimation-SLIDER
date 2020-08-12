/*
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE.txt', which is part of this source code package.
 */


//------------------------------------------------------------------------------------------------
//
//!	@file	kinAndDyn.cpp
//!     @date 2019-04-25 K.WANG & Z.H. JIANG created
//!
//!		@adapted from 2012-12-16 P. KRYCZKA
//
//------------------------------------------------------------------------------------------------
//#include "stdafx.h"
#define CONANDEF

#ifdef __QNX__
	#include <pthread.h>
	#include "Thread.h"
#else
	#include <boost/thread.hpp>
#endif

#include <ostream>
#include "kinAndDyn.h"
#include "robot.h"
#include "loadCSV.h"
#include <fstream>
#include "error.h"
//#include "ClockTimer.h"
#include "TimeLogger.h"

extern CTimeLogger dfTimeLog;

using namespace Eigen;
using namespace ROBOT;
using namespace std;

/**
  * @brief  . 
  */
  
	CkinAndDyn::CkinAndDyn():
		m_isInitialized(false)
	{ 
	pelvToLHip << 0, hipToPelvis_dist, 0;
	pelvToRHip << 0, -hipToPelvis_dist, 0;
	mirrorReflection << 1, 0, 0, 0, -1, 0, 0, 0, 1;
	int m_numberOfTrajectoryPoints = 10;
	
	// error = new Cerror;
	}

	CkinAndDyn::~CkinAndDyn()
	{ 
		// save(m_globCoM_pos,"./kindyn/m_globCoM_pos.csv");
		// save(m_globCoM_vel,"./kindyn/m_globCoM_vel.csv");
		// save(m_globCoM_acc,"./kindyn/m_globCoM_acc.csv");
		// save(zmp_multiBody,"./kindyn/zmp_multiBody.csv");
		cout<< "Joint trajectory saved "<< endl;
		save(&m_q,"./kindyn/m_q.csv");
	}
	
bool CkinAndDyn::init(int i, double stepTime) 
{
	
	m_T = stepTime;
	m_numberOfTrajectoryPoints = i;
	m_globCoM_pos = MatrixXd::Zero(3,m_numberOfTrajectoryPoints);
	m_globCoM_vel = MatrixXd::Zero(3,m_numberOfTrajectoryPoints);
	m_globCoM_acc = MatrixXd::Zero(3,m_numberOfTrajectoryPoints);
	zmp_multiBody = MatrixXd::Zero(2,m_numberOfTrajectoryPoints);

   // index: 0 pelvis, 1 left leg, 2 left foot, 3 right leg, 4 right foot
	m_indivCoM_pos.resize(m_numberOfTrajectoryPoints,MatrixXd::Zero(3,5));
	m_indivCoM_vel.resize(m_numberOfTrajectoryPoints,MatrixXd::Zero(3,5));
	m_indivCoM_acc.resize(m_numberOfTrajectoryPoints,MatrixXd::Zero(3,5));
	m_pelvis_pos.resize(m_numberOfTrajectoryPoints,Vector3d::Zero());
    m_pelvis_rot.resize(m_numberOfTrajectoryPoints,Matrix3d::Zero());
	
	m_q = MatrixXd::Zero(14,m_numberOfTrajectoryPoints);
	
	if (m_T > 0)
		m_isInitialized = true;
	else {
		error.add("Tried to set the step time to 0s");
		m_isInitialized = false;
	}
	
	return true;
}

bool CkinAndDyn::isInit() 
{
	return m_isInitialized;
}
/* void CkinAndDyn::testPrint() 
{
	Vector3d pelvis_pos;
	Matrix3d pelvis_rot;
	VectorXd lFoot(6), rFoot(6); 
	Matrix3d foot_rot; 
	whichLeg leg;
	VectorXd q(6);

	//Takalib::Util::CClockTimer time;
	pelvis_rot <<
     1,     0 ,    0,
     0,     1,     0,
     0,     0,     1;
	foot_rot <<
     1,     0,     0,
     0 ,    1 ,    0,
     0  ,   0  ,   1;	
	
	
	

	
	// init( 1 );
	// lFoot << 0,0,0,0,0,0;
	// rFoot << 0,0,0,0,0,0;
	// updateModel(0, pelvis_pos(0), pelvis_pos(1), pelvis_pos(2), lFoot, rFoot);

	vector<vector <double> > feetTr;
    loadCSV( &feetTr, "feet");
	vector<vector <double> > pelvisTr;
    loadCSV( &pelvisTr, "pelvis");	
	std::cout << " Updating started  " << pelvisTr.at(0).size()  << std::endl;
	int length = 10000; //pelvisTr.at(0).size()
	init( length , 0.001);
	// cout << "Time " << time.getTime_im() << endl;
	// timeLogger.logTime(1);
	for (int i = 0; i < length; i++) { //pelvisTr.at(0).size()
		int it=i;
		// cout << "Time " << time.getTime_im() << endl;
		// timeLogger.logTime(3);
		pelvis_pos << pelvisTr[0][it], pelvisTr[1][it], pelvisTr[2][it];
		lFoot << feetTr[0][it], feetTr[1][it], feetTr[2][it],feetTr[3][it], feetTr[4][it], feetTr[5][it];
		rFoot << feetTr[6][it], feetTr[7][it], feetTr[8][it],feetTr[9][it], feetTr[10][it], feetTr[11][it];
		// timeLogger.logTime(4);
		updateModel(i, pelvis_pos(0), pelvis_pos(1), pelvis_pos(2), lFoot, rFoot);
		// timeLogger.logTime(4);
		// timeLogger.logTime(3);
	}
	// timeLogger.logTime(1);
	// timeLogger.logTime(2);
	for (int i = 0; i < length; i++) { //pelvisTr.at(0).size()
		int it=i;
		zmp_multiBody.block(0,it,2,1) = zmpFromMultibody( m_indivCoM_pos[it], m_indivCoM_acc[it] );
	}	
	// timeLogger.logTime(2);
	
	// timeLogger.save("time.csv");

	std::cout << " Updating finished  " << std::endl;
	
	ofstream	file;	
	file.open("CoM.csv");
	file.precision(12);
	for ( int i=0; i < m_indivCoM_pos.size(); i++ )  {
		for (int j = 0; j<7; j++)
		file << m_indivCoM_pos[i](0,j) << ", " << m_indivCoM_pos[i](1,j) << ", " << m_indivCoM_pos[i](2,j) << ", ";
		file << endl;
	}

	file.close();

	file.open("CoMvel.csv");
	for ( int i=0; i < m_indivCoM_vel.size(); i++ )  {
		for (int j = 0; j<7; j++)
		file << m_indivCoM_vel[i](0,j) << ", " << m_indivCoM_vel[i](1,j) << ", " << m_indivCoM_vel[i](2,j) << ", ";
		file << endl;
	}
	file.close();	
	
	file.open("CoMacc.csv");
	for ( int i=0; i < m_indivCoM_acc.size(); i++ )  {
		for (int j = 0; j<7; j++)
		file << m_indivCoM_acc[i](0,j) << ", " << m_indivCoM_acc[i](1,j) << ", " << m_indivCoM_acc[i](2,j) << ", ";
		file << endl;
	}
	file.close();
	
	file.open("q.csv");
	for ( int i=0; i < m_q.cols(); i++ )  {
		file << m_q.block(0,i,12,1).transpose();
		file << endl;
	}
	file.close();	
	
	file.open("zmp.csv");
	for ( int i=0; i < zmp_multiBody.cols(); i++ )  {
		file << zmp_multiBody(0,i) << ", " << zmp_multiBody(1,i) << endl;
	}
	file.close();	
	
} */


bool CkinAndDyn::get_pelvisPos_given_CoMPos(Vector3d initPelvisPos, Vector3d globCoMPosRef, VectorXd lFoot, VectorXd rFoot, Vector3d *pelvisOut)
{
	
	Vector3d _error(0,0,0);
	Vector3d globalCOMpos;
	
	
	if (!calcCOM(initPelvisPos, lFoot, rFoot, &globalCOMpos)) {
		error.add("Failed in get_PelvisPos");
		return false;
	}
	_error = globCoMPosRef - globalCOMpos;	

	initPelvisPos += _error;
	int i = 0;
	while (_error.norm() > 0.0001) {
	
		if (!calcCOM(initPelvisPos, lFoot, rFoot, &globalCOMpos)) {
			error.add("Failed in get_PelvisPos");
			return false;
		}
		_error = globCoMPosRef -  globalCOMpos;
		initPelvisPos += _error;
	}
	*pelvisOut = initPelvisPos;
	return true;
}

 // foot pos here is actually the ankle pos

int  CkinAndDyn::singleLegIK( Vector3d pelvis_pos, Matrix3d pelvis_rot, Vector3d foot_pos, Matrix3d foot_rot, whichLeg leg, VectorXd *q, Vector3d *legCOMLocal)
{
	static Vector3d pelvisToHip_pos;
	static double footToHip_length;
	static double rx, ry, rz, rxz;
	static double q1=0, q2=0, q3=0, q4=0, q5=0, q6=0, q7=0;
	static double alpha, gama, costheta;
	static Matrix3d R34, R45, R56, Rot;
	static Matrix3d footToWorld_rot, hipToFoot_rot;
	static Vector3d footToHip_pos, tempLegCOM;
	static double legCOMToHipLength, tempX, tempY, tempZ, rx_offset, ry_offset, rz_offset;



    if (leg == ROBOT::left)
        pelvisToHip_pos << 0.0,  hipToPelvis_dist, 0.0;
    else if (leg == ROBOT::right)
        pelvisToHip_pos << 0.0, -hipToPelvis_dist, 0.0;

//    cout<< "Pelvis position is " << pelvis_pos << endl;

    // Foot to hip vector calculation. Vector expressed in the foot CF    
    footToWorld_rot = foot_rot.transpose();
    footToHip_pos = footToWorld_rot * (pelvis_pos +  pelvis_rot * pelvisToHip_pos - foot_pos);
   
	rx = footToHip_pos(0);
	ry = footToHip_pos(1);
	rz = footToHip_pos(2);
	
	footToHip_length = footToHip_pos.norm();
/*
	if (leg == ROBOT::left){
//		cout << "Left leg length " << c << endl;
		//cout << "Left foot position " << foot_pos << endl;
		//cout << "Left pelvis position " << pelvis_pos << endl;
		//cout << "Left hip position " << (pelvis_pos +  pelvis_rot * pelvisToHip_pos)<< endl;
		cout << "Left footToHip position " << footToHip_pos << endl;
	}
	else if (leg == ROBOT::right){
//		cout << "Right leg length " << c << endl;
		//cout << "Right foot position " << foot_pos << endl;
		//cout << "Right pelvis position " << pelvis_pos << endl;
		//cout << "RIght hip position " << (pelvis_pos +  pelvis_rot * pelvisToHip_pos)<< endl;
		cout << "Right footToHip position " << footToHip_pos << endl;
		}
*/
    
	// Check if the knee is not overstretched     
    if ( footToHip_length >= leg_len ) {
    	cout << "Knee close to full extension" << endl << flush;
    	cout << "footToHip_length = " << footToHip_length << endl << flush;
    	cout << "leg_len = " << leg_len << endl << flush;
    	cout << "pelvis_pos = " << pelvis_pos << endl << flush;
    	cout << "foot_pos = " << foot_pos << endl << flush;
        error.add("Knee close to full extension");
		//return 1;
    } // obsolete because no knees on SLIDER

	
    // Ankle roll
    rxz = sqrt(rx*rx + rz*rz);
	q5 = atan2(ry, rxz);
	
    // Ankle pitch
	q6 = -(atan2(rx,rz));
	
	//cout << "sin(0.1): " << sin(0.1) << endl << std::flush;
	//cout << "q6: " << q6 << endl << std::flush;
	//cout << "-sin(q6): " << -sin(q6) << endl << std::flush;
	//cout << "roty(q6)(2,0): \n" << roty(q6) << endl << std::flush;
	//cout << "roty(q6)(2,0): " << roty(q6)(2,0) << endl << std::flush;

    // Hip joint    
	R45 = rotx(q5);
	R56 = roty(q6);

	hipToFoot_rot = (pelvis_rot.transpose())*foot_rot;

	Rot = hipToFoot_rot*((R45*R56).transpose());
	
    // Hip pitch
	q1 = atan2(Rot(0,2), Rot(2,2)); 

	//cout << "q1: " << q1 << endl << std::flush;
	/*
    cout << "R45(2,0): " << (R45)(2,0) << endl << std::flush;
    cout << "R45(2,1): " << (R45)(2,1) << endl << std::flush;
    cout << "R45(2,2): " << (R45)(2,2) << endl << std::flush;
    cout << "R56(0,0): " << (R56)(0,0) << endl << std::flush;
    cout << "R56(1,0): " << (R56)(1,0) << endl << std::flush;
    cout << "R56(2,0): " << (R56)(2,0) << endl << std::flush;
    */

	// Hip roll
	q2 = asin(-Rot(1,2));

    // Hip yaw
	q3 = atan2(Rot(1,0), Rot(1,1)); 

	// Add the offset for old SLIDER
	q7 = asin(ankle_inner_offset/footToHip_length);


    // hip slide for new SLIDER (no ankle_inner_offset), upper side of the leg
//	q4 = leg_len - footToHip_length; // I don't think we need ankle_inner_offset to get q4 here. This q4 is the upper part the leg above the hip
    
//	q4 = footToHip_length; // actually the distance from ankle to hip
	q4 = footToHip_length;

//	cout<< "slide value " << q4 << endl;
    (*q)(0) = q1;
    (*q)(1) = q2;
    (*q)(2) = q3;
    (*q)(3) = q4;
	(*q)(4) = q5;
    (*q)(5) = q6; 
    (*q)(6) = q7;

    // Calcuate the local pos of COM of the leg w.r.t the hip
    legCOMToHipLength = fabs(leg_len/2 - q4);
    // Compute local coordinates of leg COM with respect to leg coordinate
    tempX = legCOMToHipLength*rx/footToHip_length;
    tempY = legCOMToHipLength*ry/footToHip_length;
    tempZ = legCOMToHipLength*rz/footToHip_length;
    (*legCOMLocal) << tempX,tempY,tempZ;
	
return 0;
}

// ------------------------------------------------------------------------------------------------
// Added from onlinepg_opt
int CkinAndDyn::calcPelvisToFootFK(Vector3d pelvis_pos, Matrix3d pelvis_rot, VectorXd q, whichLeg foot,
                       Vector3d *ankleJoint_pos, Matrix3d *ankle_rot)
{
    Vector3d hipJoint_pos;
    if (foot == ROBOT::left) {
        pelvToLHip << 0, hipToPelvis_dist, 0;
		hipJoint_pos = pelvis_pos +  pelvis_rot * pelvToLHip;
	} else {
		pelvToRHip << 0, -hipToPelvis_dist, 0;
        hipJoint_pos = pelvis_pos +  pelvis_rot * pelvToRHip;
	}

	Vector3d ankleLocal;
	ankleLocal << 0,0, -q(3);

    Matrix3d hipPitch_rot = roty(q(0));
    Matrix3d hipRoll_rot = rotx(q(1));
    Matrix3d hipYaw_rot = rotz(q(2));
    Matrix3d pelvisToHip_rot = hipPitch_rot * hipRoll_rot * hipYaw_rot;

    // 
    *ankleJoint_pos = hipJoint_pos + pelvis_rot*pelvisToHip_rot*ankleLocal;
    Matrix3d anklePitch_rot = roty(q(5));
    Matrix3d ankleRoll_rot = rotx(q(4));
    Matrix3d HipToFoot_rot = ankleRoll_rot * anklePitch_rot;
    *ankle_rot = pelvis_rot*pelvisToHip_rot*HipToFoot_rot;
    return 0;
}   

// I didn't add calcFootToPelvisFK, which I think will not be used.

Vector3d CkinAndDyn::calcCOMLocPos(Matrix3d pelvisRot, VectorXd q, whichLeg stanceFoot, Vector3d leftLegCOMToHip, Vector3d rightLegCOMToHip)
{
	Vector3d pelvis_pos = Vector3d::Zero();
	Vector3d ankleJoint_pos = Vector3d::Zero();
	Matrix3d ankle_rot = Matrix3d::Zero();

	//Chose support leg
	VectorXd qLeg; 
	VectorXd qL = q.segment(7,7);
	VectorXd qR = q.segment(0,7);  

	// Calculate global COM assuming that pelvis is at (0,0,0) and rotation is taken from IMU. 
	Vector3d wholeBodyCOMpos = calcCOM(pelvis_pos, pelvisRot, qL, qR, leftLegCOMToHip, rightLegCOMToHip);	

    if (stanceFoot == ROBOT::left) 
    {
        qLeg = q.segment(7,7);
    }
    else
    {
        qLeg = q.segment(0,7);
    }

	//What is position of ankle with rescpect to (0,0,0)?
	calcPelvisToFootFK( pelvis_pos, pelvisRot, qLeg, stanceFoot, &ankleJoint_pos, &ankle_rot);
	
	Vector3d localCOMpos = wholeBodyCOMpos - ankleJoint_pos; 
	
	return localCOMpos;
}

Vector3d CkinAndDyn::calcCOM(Vector3d pelvis_pos, Matrix3d pelvis_rot, VectorXd qL, VectorXd qR, Vector3d leftLegCOMToHip, Vector3d rightLegCOMToHip)
{
	Matrix<double, 3, 7> COM_pos;

	Vector3d leftHipJoint_pos, leftAnkleJoint_pos;
	Vector3d rightHipJoint_pos, rightAnkleJoint_pos;
	Vector3d leftAnkleLocal, rightAnkleLocal;

	Matrix3d hipPitch_rot, hipRoll_rot, hipYaw_rot, pelvisToHip_rot, HipToFoot_rot;
	Matrix3d anklePitch_rot, ankleRoll_rot;
	Matrix3d leftAnkle_rot, rightAnkle_rot;

	leftAnkleLocal << 0,0, -qL(3);
	rightAnkleLocal << 0,0, -qR(3);

	// Pelvis and upper body CoM 
	// COM_pos.at(i).block(0,0,3,1) = pelvis_pos + pelvis_rot * upperBody_CoM;

	// COM position of plevis in global coordinate
	COM_pos.block<3,1>(0,0) = pelvis_pos + pelvis_rot * upperBody_CoM;

	// ------------------------------- Left leg -------------------------------
	// Left leg CoM in global coordinate
	leftHipJoint_pos = pelvis_pos +  pelvis_rot * pelvToLHip;
	hipPitch_rot = roty(qL(0));
	hipRoll_rot = rotx(qL(1));
	hipYaw_rot = rotz(qL(2));
	pelvisToHip_rot = hipPitch_rot * hipRoll_rot * hipYaw_rot;
	COM_pos.block<3,1>(0,1) = leftHipJoint_pos - leftLegCOMToHip;

	// Left foot CoM
	leftAnkleJoint_pos = leftHipJoint_pos + pelvis_rot*pelvisToHip_rot*leftAnkleLocal;
	anklePitch_rot = roty(qL(5));
	ankleRoll_rot = rotx(qL(4));
	HipToFoot_rot = ankleRoll_rot * anklePitch_rot;
	COM_pos.block<3,1>(0,2) = leftAnkleJoint_pos + pelvis_rot*pelvisToHip_rot*HipToFoot_rot*mirrorReflection*foot_CoM;
	leftAnkle_rot = pelvis_rot*pelvisToHip_rot*HipToFoot_rot;

	// ------------------------------- Right leg -------------------------------
	// Right leg CoM in global coordinate
	rightHipJoint_pos = pelvis_pos +  pelvis_rot * pelvToRHip;
	hipPitch_rot = roty(qR(0));
	hipRoll_rot = rotx(qR(1));
	hipYaw_rot = rotz(qR(2));
	pelvisToHip_rot = hipPitch_rot * hipRoll_rot * hipYaw_rot;
	COM_pos.block<3,1>(0,3) = rightHipJoint_pos - rightLegCOMToHip;

	// Right foot CoM
	rightAnkleJoint_pos = rightHipJoint_pos + pelvis_rot*pelvisToHip_rot*rightAnkleLocal;
	anklePitch_rot = roty(qR(5));
	ankleRoll_rot = rotx(qR(4));
	HipToFoot_rot = ankleRoll_rot * anklePitch_rot;
	COM_pos.block<3,1>(0,4) = rightAnkleJoint_pos + pelvis_rot*pelvisToHip_rot*HipToFoot_rot*foot_CoM;
	rightAnkle_rot = pelvis_rot*pelvisToHip_rot*HipToFoot_rot;

	 
	// ------------------------------- Whole body CoM --------------------------
	Vector3d globalCOMpos;
	globalCOMpos =(upperBody_mass*COM_pos.block<3,1>(0,0) +
								   leg_mass*(COM_pos.block<3,1>(0,1) + COM_pos.block<3,1>(0,3))+
								   foot_mass*(COM_pos.block<3,1>(0,2) + COM_pos.block<3,1>(0,4))) /
								   (total_mass);

	return globalCOMpos;
}


bool  CkinAndDyn::calcCOM(Vector3d worldToPelvis_pos, VectorXd lFoot, VectorXd rFoot, Vector3d *globalCOMpos)
{
		
	Matrix3d identity = Matrix3d::Identity();
	VectorXd qL(7), qR(7); 
	Vector3d leftLegCOMToHip, rightLegCOMToHip;

	
	// Inverse kinematics
	if (singleLegIK(worldToPelvis_pos, identity, lFoot.head(3), identity, ROBOT::left, &qL, &leftLegCOMToHip) != 0) {
		string errorTmp;
		errorTmp = "There was a problem with inverse kinematics of the left foot. Index ";
			
		error.add(errorTmp); 
		return false;
	}
	
	if (singleLegIK(worldToPelvis_pos, identity, rFoot.head(3), identity, ROBOT::right, &qR, &rightLegCOMToHip) != 0) {
		string errorTmp;
		errorTmp = "There was a problem with inverse kinematics of the right foot. Index ";
			
		error.add(errorTmp); 
		return false;
	} 

	VectorXd q(14);
	q.segment(0,7) = qR;
	q.segment(7,7) = qL;
	
	*globalCOMpos = CkinAndDyn::calcCOM(worldToPelvis_pos, identity, qL, qR, leftLegCOMToHip, rightLegCOMToHip);
	
	return globalCOMpos;
}



// ------------------------------------------------------------------------------------------------

   
// --------------------------------------------------------------------------------------------------
bool  CkinAndDyn::updateModel(int index, double worldToPelvis_x, double worldToPelvis_y, double worldToPelvis_z, VectorXd lFoot, VectorXd rFoot)
{
	static Vector3d worldToPelvis_pos;
	worldToPelvis_pos(0) = worldToPelvis_x;
	worldToPelvis_pos(1) = worldToPelvis_y;
	worldToPelvis_pos(2) = worldToPelvis_z;
	return updateModel(index, worldToPelvis_pos, lFoot, rFoot);
}

bool  CkinAndDyn::updateModel(int index, Vector3d worldToPelvis_pos, VectorXd lFoot, VectorXd rFoot)
 {
	if ( index >= m_numberOfTrajectoryPoints) {
		error.add("Tried to update the model point which is out of range.");
		cout << "Tried to update the model point which is out of range." << endl << flush;
		return false;
	}
	
	if (m_isInitialized == false) {
		error.add("Tried to use uninitialized instance of the class.");
		cout << "Tried to use uninitialized instance of the class." << endl << flush;
		return false;
	}
	
	Matrix3d worldToPelvis_rot;
	// each side of leg has 7 parameters: 1-3 is hip joints(pitch, roll, yaw), 4 is slide prismatic change
	// 5-6 is ankle pitch & roll, 7 is ankle inner offset
	VectorXd qL(7), qR(7);
	Vector3d leftLegCOMToHip, rightLegCOMToHip;
	Matrix3d leftFoot_rot, rightFoot_rot; //RY, RZ,
	
	// Pelvis orientation and position
	worldToPelvis_rot = rotz( (lFoot(5) + rFoot(5)) / 2.0 ); // Modify this way of calculation. It has singularity point
	
	// Foot rotation     
	leftFoot_rot = rotz(lFoot(3)) * roty(lFoot(4));

	// Foot rotation
	rightFoot_rot = rotz(rFoot(3)) * roty(rFoot(4));    

	// Inverse kinematics
	if (singleLegIK(worldToPelvis_pos, worldToPelvis_rot, lFoot.head(3), leftFoot_rot, ROBOT::left, &qL, &leftLegCOMToHip) != 0) {
		ostringstream indexNumber;
		indexNumber << index;
		
		string errorTmp;
		errorTmp = "There was a problem with inverse kinematics of the left foot. Index ";
		errorTmp +=  indexNumber.str();
			
		error.add(errorTmp); 
		cout << errorTmp << endl << flush;
		return false;
	}
	
	if (singleLegIK(worldToPelvis_pos, worldToPelvis_rot, rFoot.head(3), rightFoot_rot, ROBOT::right, &qR, &rightLegCOMToHip) != 0) {
		ostringstream indexNumber;
		indexNumber << index;
		
		string errorTmp;
		errorTmp = "There was a problem with inverse kinematics of the right foot. Index ";
		errorTmp +=  indexNumber.str();
			
		error.add(errorTmp); 
		cout << errorTmp << endl << flush;
		return false;
	} 
	
	// Calculate new CoM position
	if (updateCoM(index,worldToPelvis_pos, worldToPelvis_rot, qL, qR, leftLegCOMToHip, rightLegCOMToHip) != 0) {

		ostringstream indexNumber;
		indexNumber << index;
		
		string errorTmp;
		errorTmp = "Some problem with individual mass calculation. Index ";
		errorTmp +=  indexNumber.str();
			
		error.add(errorTmp); 
		cout << errorTmp << endl << flush;
		return false;	
	} 
	// Multibody dynamics
	
	if (index>0) { // Skip the first iteration 
		m_indivCoM_vel[index] = ( m_indivCoM_pos[index] - m_indivCoM_pos[index-1] ) / m_T;
		m_globCoM_vel.block<3,1>(0,index) = ( m_globCoM_pos.block<3,1>(0,index) - m_globCoM_pos.block<3,1>(0,index-1) ) / m_T;

		if (index>1) {
			m_indivCoM_acc[index] = ( m_indivCoM_vel[index] - m_indivCoM_vel[index-1] ) / m_T;
			m_globCoM_acc.block<3,1>(0,index) = ( m_globCoM_vel.block<3,1>(0,index) - m_globCoM_vel.block<3,1>(0,index-1) ) / m_T;          
			zmp_multiBody.block<2,1>(0,index) = zmpFromMultibody( m_indivCoM_pos[index], m_indivCoM_acc[index] );

			if ( (zmp_multiBody.block<1,1>(0,index)(0,0) != zmp_multiBody.block<1,1>(0,index)(0,0)) && 
				 (zmp_multiBody.block<3,1>(1,index)(0,0) != zmp_multiBody.block<3,1>(1,index)(0,0))) { 
				cout << "CoM pos: " << m_indivCoM_pos[index] << endl;
				cout << "CoM acc: " << m_indivCoM_acc[index] << endl;
				error.add("ZMP calculation failed. Something is wrong!!");
				cout << "ZMP calculation failed. Something is wrong!!" << endl << flush;
				return false;
			}
		}
	}
	
	m_pelvis_pos.at(index) = worldToPelvis_pos;	
	m_pelvis_rot.at(index) = worldToPelvis_rot;

	m_q.block<7,1>(0,index)= qR;
	m_q.block<7,1>(7,index)= qL;
	//cout << "here" << endl << std::flush;


	return true;
}


bool  CkinAndDyn::updateModel_Harry(int index, Vector3d worldToPelvis_pos, VectorXd lFoot, VectorXd rFoot, Vector3d *leftLegCOMToHip, Vector3d *rightLegCOMToHip)
 {
	if ( index >= m_numberOfTrajectoryPoints) {
		error.add("Tried to update the model point which is out of range.");
		return false;
	}
	
	if (m_isInitialized == false) {
		error.add("Tried to use uninitialized instance of the class.");
		return false;
	}
	
	Matrix3d worldToPelvis_rot;
	// each side of leg has 7 parameters: 1-3 is hip joints(pitch, roll, yaw), 4 is slide prismatic change
	// 5-6 is ankle pitch & roll, 7 is ankle inner offset
	VectorXd qL(7), qR(7);
	Matrix3d leftFoot_rot, rightFoot_rot; //RY, RZ,
	
	// Pelvis orientation and position
	worldToPelvis_rot = rotz( (lFoot(5) + rFoot(5)) / 2.0 ); // Modify this way of calculation. It has singularity point
	
	// Foot rotation     
	leftFoot_rot = rotz(lFoot(3)) * roty(lFoot(4));

	// Foot rotation
	rightFoot_rot = rotz(rFoot(3)) * roty(rFoot(4));    

	// Inverse kinematics
	if (singleLegIK(worldToPelvis_pos, worldToPelvis_rot, lFoot.head(3), leftFoot_rot, ROBOT::left, &qL, leftLegCOMToHip) != 0) {
		ostringstream indexNumber;
		indexNumber << index;
		
		string errorTmp;
		errorTmp = "There was a problem with inverse kinematics of the left foot. Index ";
		errorTmp +=  indexNumber.str();
			
		error.add(errorTmp); 
		return false;
	}
	
	if (singleLegIK(worldToPelvis_pos, worldToPelvis_rot, rFoot.head(3), rightFoot_rot, ROBOT::right, &qR, rightLegCOMToHip) != 0) {
		ostringstream indexNumber;
		indexNumber << index;
		
		string errorTmp;
		errorTmp = "There was a problem with inverse kinematics of the right foot. Index ";
		errorTmp +=  indexNumber.str();
			
		error.add(errorTmp); 
		return false;
	} 
	
	// Calculate new CoM position
	if (updateCoM(index,worldToPelvis_pos, worldToPelvis_rot, qL, qR, *leftLegCOMToHip, *rightLegCOMToHip) != 0) {

		ostringstream indexNumber;
		indexNumber << index;
		
		string errorTmp;
		errorTmp = "Some problem with individual mass calculation. Index ";
		errorTmp +=  indexNumber.str();
			
		error.add(errorTmp); 
		return false;	
	} 
	// Multibody dynamics
	
	if (index>0) { // Skip the first iteration 
		m_indivCoM_vel[index] = ( m_indivCoM_pos[index] - m_indivCoM_pos[index-1] ) / m_T;
		m_globCoM_vel.block<3,1>(0,index) = ( m_globCoM_pos.block<3,1>(0,index) - m_globCoM_pos.block<3,1>(0,index-1) ) / m_T;

		if (index>1) {
			m_indivCoM_acc[index] = ( m_indivCoM_vel[index] - m_indivCoM_vel[index-1] ) / m_T;
			m_globCoM_acc.block<3,1>(0,index) = ( m_globCoM_vel.block<3,1>(0,index) - m_globCoM_vel.block<3,1>(0,index-1) ) / m_T;          
			zmp_multiBody.block<2,1>(0,index) = zmpFromMultibody( m_indivCoM_pos[index], m_indivCoM_acc[index] );

			if ( (zmp_multiBody.block<1,1>(0,index)(0,0) != zmp_multiBody.block<1,1>(0,index)(0,0)) && 
				 (zmp_multiBody.block<3,1>(1,index)(0,0) != zmp_multiBody.block<3,1>(1,index)(0,0))) { 
				cout << "CoM pos: " << m_indivCoM_pos[index] << endl;
				cout << "CoM acc: " << m_indivCoM_acc[index] << endl;
				error.add("ZMP calculation failed. Something is wrong!!");
				return false;
			}
		}
	}
	
	m_pelvis_pos.at(index) = worldToPelvis_pos;	
	m_pelvis_rot.at(index) = worldToPelvis_rot;

	m_q.block<7,1>(0,index)= qR;
	m_q.block<7,1>(7,index)= qL;
	//cout << "here" << endl << std::flush;


	return true;
}




// --------------------------------------------------------------------------------------------------
int  CkinAndDyn::updateCoM(int i, Vector3d pelvis_pos, Matrix3d pelvis_rot, VectorXd qL, VectorXd qR, Vector3d leftLegCOMToHip, Vector3d rightLegCOMToHip)
{

	Vector3d leftHipJoint_pos, leftAnkleJoint_pos;
	Vector3d rightHipJoint_pos, rightAnkleJoint_pos;
	Vector3d leftAnkleLocal, rightAnkleLocal;

	Matrix3d hipPitch_rot, hipRoll_rot, hipYaw_rot, pelvisToHip_rot, HipToFoot_rot;
	Matrix3d anklePitch_rot, ankleRoll_rot;
	Matrix3d leftAnkle_rot, rightAnkle_rot;

	leftAnkleLocal << 0,0, -qL(3);
	rightAnkleLocal << 0,0, -qR(3);
	// Pelvis and upper body CoM 
	// m_indivCoM_pos.at(i).block(0,0,3,1) = pelvis_pos + pelvis_rot * upperBody_CoM;

	// COM position of plevis in global coordinate
	m_indivCoM_pos.at(i).block<3,1>(0,0) = pelvis_pos + pelvis_rot * upperBody_CoM;

	// ------------------------------- Left leg -------------------------------
	// Left leg CoM in global coordinate
	leftHipJoint_pos = pelvis_pos +  pelvis_rot * pelvToLHip;
	hipPitch_rot = roty(qL(0));
	hipRoll_rot = rotx(qL(1));
	hipYaw_rot = rotz(qL(2));
	pelvisToHip_rot = hipPitch_rot * hipRoll_rot * hipYaw_rot;
	m_indivCoM_pos.at(i).block<3,1>(0,1) = leftHipJoint_pos - leftLegCOMToHip;

	// Left foot CoM
	leftAnkleJoint_pos = leftHipJoint_pos + pelvis_rot*pelvisToHip_rot*leftAnkleLocal;
	anklePitch_rot = roty(qL(5));
	ankleRoll_rot = rotx(qL(4));
	HipToFoot_rot = ankleRoll_rot * anklePitch_rot;
	m_indivCoM_pos.at(i).block<3,1>(0,2) = leftAnkleJoint_pos + pelvis_rot*pelvisToHip_rot*HipToFoot_rot*mirrorReflection*foot_CoM;
	leftAnkle_rot = pelvis_rot*pelvisToHip_rot*HipToFoot_rot;

	// ------------------------------- Right leg -------------------------------
	// Right leg CoM in global coordinate
	rightHipJoint_pos = pelvis_pos +  pelvis_rot * pelvToRHip;
	hipPitch_rot = roty(qR(0));
	hipRoll_rot = rotx(qR(1));
	hipYaw_rot = rotz(qR(2));
	pelvisToHip_rot = hipPitch_rot * hipRoll_rot * hipYaw_rot;
	m_indivCoM_pos.at(i).block<3,1>(0,3) = rightHipJoint_pos - rightLegCOMToHip;

	// Right foot CoM
	rightAnkleJoint_pos = rightHipJoint_pos + pelvis_rot*pelvisToHip_rot*rightAnkleLocal;
	anklePitch_rot = roty(qR(5));
	ankleRoll_rot = rotx(qR(4));
	HipToFoot_rot = ankleRoll_rot * anklePitch_rot;
	m_indivCoM_pos.at(i).block<3,1>(0,4) = rightAnkleJoint_pos + pelvis_rot*pelvisToHip_rot*HipToFoot_rot*foot_CoM;
	rightAnkle_rot = pelvis_rot*pelvisToHip_rot*HipToFoot_rot;
	 
	// ------------------------------- Whole body CoM --------------------------
	m_globCoM_pos.block<3,1>(0,i) =(upperBody_mass*m_indivCoM_pos.at(i).block<3,1>(0,0) +
								   leg_mass*(m_indivCoM_pos.at(i).block<3,1>(0,1) + m_indivCoM_pos.at(i).block<3,1>(0,3))+
								   foot_mass*(m_indivCoM_pos.at(i).block<3,1>(0,2) + m_indivCoM_pos.at(i).block<3,1>(0,4))) /
								   (total_mass);

	return 0;
}


bool  CkinAndDyn::updateModelNoMBS(int index, Vector3d worldToPelvis_pos, VectorXd lFoot, VectorXd rFoot)
 {
 	Vector3d leftLegCOMToHip, rightLegCOMToHip;
	 
	if ( index >= m_numberOfTrajectoryPoints) {
		error.add("Tried to update the model point which is out of range.");
		return false;
	}
	
	if (m_isInitialized == false) {
		error.add("Tried to use uninitialized instance of the class.");
		return false;
	}
	
	Matrix3d worldToPelvis_rot;
	VectorXd qL(7), qR(7); 
	Matrix3d leftFoot_rot, rightFoot_rot; //RY, RZ,


	
	// Pelvis orientation and position
	worldToPelvis_rot = rotz( (lFoot(5) + rFoot(5)) / 2.0 ); // Modify this way of calculation. It has singularity point

	
	// Foot rotation     
	leftFoot_rot = rotz(lFoot(3)) * roty(lFoot(4));

	// Foot rotation
	rightFoot_rot = rotz(rFoot(3)) * roty(rFoot(4));    
	
	// Inverse kinematics
	if (singleLegIK(worldToPelvis_pos, worldToPelvis_rot, lFoot.head(3), leftFoot_rot, ROBOT::left, &qL, &leftLegCOMToHip) != 0) {
		ostringstream indexNumber;
		indexNumber << index;
		
		string errorTmp;
		errorTmp = "There was a problem with inverse kinematics of the left foot. Index ";
		errorTmp +=  indexNumber.str();
			
		error.add(errorTmp); 
		return false;
	}
	
	if (singleLegIK(worldToPelvis_pos, worldToPelvis_rot, rFoot.head(3), rightFoot_rot, ROBOT::right, &qR, &rightLegCOMToHip) != 0) {
		ostringstream indexNumber;
		indexNumber << index;
		
		string errorTmp;
		errorTmp = "There was a problem with inverse kinematics of the right foot. Index ";
		errorTmp +=  indexNumber.str();
			
		error.add(errorTmp); 
		return false;
	} 
	

	// Calculate new CoM position
	if (updateCoM(index,worldToPelvis_pos, worldToPelvis_rot, qL, qR, leftLegCOMToHip, rightLegCOMToHip) != 0) {
		ostringstream indexNumber;
		indexNumber << index;
		
		string errorTmp;
		errorTmp = "Some problem with individual mass calculation. Index ";
		errorTmp +=  indexNumber.str();
			
		error.add(errorTmp); 
		return false;	
	} 
	
	m_pelvis_pos.at(index) = worldToPelvis_pos;
	m_pelvis_rot.at(index) = worldToPelvis_rot;

	m_q.block<7,1>(0,index)= qR;
	m_q.block<7,1>(7,index)= qL;	

	return true;
}



bool CkinAndDyn::replicateZMP(int startingPoint)
{
	if (startingPoint >= zmp_multiBody.cols()) return false;
	
	int numberOfPoints = zmp_multiBody.cols() - startingPoint;
	zmp_multiBody.block(0, startingPoint, 2, numberOfPoints) = zmp_multiBody.block(0, startingPoint, 2, 1).replicate(1,numberOfPoints);
	
	return true;
}


Vector2d CkinAndDyn::zmpFromMultibody( MatrixXd CoG_pos, MatrixXd CoG_acc )
{	
	//VectorXd item(6);
	VectorXd item(5);

	/*
	 * CoG_acc(0,i) is the x-component of COG accelaration of the i-th link
	 * CoG_acc(1,i) is the y-component of COG accelaration of the i-th link
	 * CoG_acc(2,i) is the z-component of COG accelaration of the i-th link
	 * CoG_pos(0,i) is the x-component of COG position of the i-th link
	 * CoG_pos(1,i) is the y-component of COG position of the i-th link
	 * CoG_pos(2,i) is the z-component of COG position of the i-th link
	*/

	item(0)=upperBody_mass*CoG_pos(0,0)*(G_const + CoG_acc(2,0) ) - upperBody_mass*CoG_pos(2,0)*CoG_acc(0,0);
	item(1)=leg_mass*CoG_pos(0,1)*(G_const + CoG_acc(2,1) )- leg_mass*CoG_pos(2,1)*CoG_acc(0,1);
	item(2)=foot_mass*CoG_pos(0,2)*(G_const + CoG_acc(2,2) )- foot_mass*CoG_pos(2,2)*CoG_acc(0,2);
	item(3)=leg_mass*CoG_pos(0,3)*(G_const + CoG_acc(2,3) )- leg_mass*CoG_pos(2,3)*CoG_acc(0,3);
	item(4)=foot_mass*CoG_pos(0,4)*(G_const + CoG_acc(2,4) )- foot_mass*CoG_pos(2,4)*CoG_acc(0,4);
	double XNumerator = item.sum(); 

	item(0)=upperBody_mass*CoG_pos(1,0)*(G_const + CoG_acc(2,0) ) - upperBody_mass*CoG_pos(2,0)*CoG_acc(1,0);
	item(1)=leg_mass*CoG_pos(1,1)*(G_const + CoG_acc(2,1) )- leg_mass*CoG_pos(2,1)*CoG_acc(1,1);
	item(2)=foot_mass*CoG_pos(1,2)*(G_const + CoG_acc(2,2) )- foot_mass*CoG_pos(2,2)*CoG_acc(1,2);
	item(3)=leg_mass*CoG_pos(1,3)*(G_const + CoG_acc(2,3) )- leg_mass*CoG_pos(2,3)*CoG_acc(1,4);
	item(4)=foot_mass*CoG_pos(1,4)*(G_const + CoG_acc(2,4) )- foot_mass*CoG_pos(2,4)*CoG_acc(1,4);
	double YNumerator =  item.sum();  

	item(0)=upperBody_mass*(G_const + CoG_acc(2,0) ) ;
	item(1)=leg_mass*(G_const + CoG_acc(2,1) );
	item(2)=foot_mass*(G_const + CoG_acc(2,2) );
	item(3)=leg_mass*(G_const + CoG_acc(2,3) );
	item(4)=foot_mass*(G_const + CoG_acc(2,4) );
	
	// for (int i=0; i<5; i++)
		// cout << "item " << i << " : " << item(i) << endl;   
	
	double Denominator =  item.sum(); 
	// cout << "Dem" << Denominator << endl;
	
	Vector2d zmp_multiBody; 
	zmp_multiBody(0) = XNumerator/Denominator;
	zmp_multiBody(1) = YNumerator/Denominator; 
	
	// cout << "ZMP " << zmp_multiBody(0) << " \t" << zmp_multiBody(1) << endl;
	
	return zmp_multiBody;

	
}


inline Matrix3d CkinAndDyn::rotx(double angle)
{
	//Matrix defined as stattic to accelerate calculation
	static Matrix3d result = Matrix3d::Identity();
	double ct = cos(angle);
	double st = sin(angle);
	
	result(1,1) = ct;
	result(1,2) = -st;
	result(2,1) = st;
	result(2,2) = ct;
	
	// result << 1, 0,	0,
	// 		  0,ct,	-st,
	// 		  0,st,	ct;
	
	return result;
}

inline Matrix3d CkinAndDyn::roty(double angle)
{
	//Matrix defined as stattic to accelerate calculation
	static Matrix3d result = Matrix3d::Identity();
	double ct = cos(angle);
	double st = sin(angle);
	
	result(0,0) = ct;
	result(0,2) = st;
	result(2,0) = -st;
	result(2,2) = ct;
	
	//cout << "angle: " << angle << endl << std::flush;
	//cout << "sin(angle): " << -sin(angle) << endl << std::flush;
	//cout << "-st: " << -st << endl << std::flush;
	//cout << "result(2,0): " << result(2,0) << endl << std::flush;

	// result  << ct,  0,  st,
	// 		    0,  1,	0,
	// 		  -st,  0,  ct;
			 
	return result;
}
inline Matrix3d CkinAndDyn::rotz(double angle)
{
	//Matrix defined as stattic to accelerate calculation
	static Matrix3d result = Matrix3d::Identity();
	double ct = cos(angle);
	double st = sin(angle);
	
	result(0,0) = ct;
	result(0,1) = -st;
	result(1,0) = st;
	result(1,1) = ct;
			   
	return result;
}
