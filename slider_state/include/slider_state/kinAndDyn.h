/*
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE.txt', which is part of this source code package.
 */


//------------------------------------------------------------------------------------------------
//
//!	@file	kinAndDyn.h
//!     @date 2019-04-25 K.WANG & Z.H. JIANG created
//!
//!		@adapted from 2012-12-16 P. KRYCZKA
//
/*! \file */ 
//------------------------------------------------------------------------------------------------
#ifndef	KINANDDYN_H
#define	KINANDDYN_H

#include "error.h"
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include "robot.h"
#include "saveEigen.h"

#define PI 3.14159265359

using namespace Eigen;

//! Handles all kinematic and dynamic information of the system. 
/*! To use:
 - create instance of the class 
 - call init()
 - on each step call updateModel() 
 - access desired data by directly accessing the matrices (I know it's a messy way :( )
 
 If you want to modify kinemaitc or dynamic model change functions: singleLegIK(), updateCoM() and zmpFromMultibody().
 
 
CAUTION:
 - Don't write to the variables available globally. Access them only to read their values!!
*/
class	CkinAndDyn
{
	/// Number of points in trajectory
	int m_numberOfTrajectoryPoints;		
	///Step time used in calcualtion of velocities and accelerations	
	double m_T; 						
	bool m_isInitialized;

	//! Calculates rotation matrix around x axis 
	inline Eigen::Matrix3d rotx(double angle);
	
	//! Calculates rotation matrix around y axis 
	inline Eigen::Matrix3d roty(double angle);
	
	//! Calculates rotation matrix around z axis 
	inline Eigen::Matrix3d rotz(double angle);

	// void testPrint ();
	
	//! COMAN inverse kinematics for single leg.
	/*! 
		@param[in] pelvis_pos - pelvis position vector in world CF
		@param[in] pelvis_rot - pelvis rotation matrix in world CF
		@param[in] foot_pos - foot position vector 
		@param[in] foot_rot - foot rotation matrix 
		@param[in] leg - L or R 
		@param[out] q - output vector of leg joint angles 
	*/
	int  singleLegIK( Eigen::Vector3d pelvis_pos, Eigen::Matrix3d pelvis_rot, Eigen::Vector3d foot_pos,
					  Eigen::Matrix3d foot_rot, ROBOT::whichLeg leg, Eigen::VectorXd *q, Eigen::Vector3d *legCOMLocal);
	
	
	//! Updates position velocity and acceleration of each link CoM. 
	/*! 
		@param[in] i - sample which should be updated
		@param[in] pelvis_pos - pelvis position vector in world CF
		@param[in] pelvis_rot - pelvis rotation matrix in world CF
		@param[in] qL - vector of left leg joint angles 
		@param[in] qR - vector of right leg joint angles 
	*/
	int  updateCoM(int i, Eigen::Vector3d pelvis_pos, Eigen::Matrix3d pelvis_rot, Eigen::VectorXd qL, Eigen::VectorXd qR, Eigen::Vector3d leftLegCOMToHip, Eigen::Vector3d rightLegCOMToHip);
	
	//! Calculates ZMP position given CoM motion information.
	/*! 
		@param[in] CoG_pos - links CoM position
		@param[in] CoG_acc - links CoM acceleration
		@retval x and y coordinates of the ZMP position resulting from MBS calculation 
	*/
	Eigen::Vector2d zmpFromMultibody( Eigen::MatrixXd CoG_pos, Eigen::MatrixXd CoG_acc );

public:

// ************************* MEMBER FUNCTIONS **********************************
	CkinAndDyn();
	~CkinAndDyn();

	int calcPelvisToFootFK(Eigen::Vector3d pelvis_pos, Eigen::Matrix3d pelvis_rot, Eigen::VectorXd q, ROBOT::whichLeg foot, Eigen::Vector3d *ankleJoint_pos, Eigen::Matrix3d *ankle_rot);

	Vector3d  calcCOM(Vector3d pelvis_pos, Matrix3d pelvis_rot, Eigen::VectorXd qL, Eigen::VectorXd qR, Eigen::Vector3d leftLegCOMToHip, Eigen::Vector3d rightLegCOMToHip);
	bool  calcCOM(Vector3d worldToPelvis_pos, VectorXd lFoot, VectorXd rFoot, Vector3d *globalCOMpos);

	Vector3d calcCOMLocPos(Matrix3d pelvisRot, VectorXd q, ROBOT::whichLeg stanceFoot, Eigen::Vector3d leftLegCOMToHip, Eigen::Vector3d rightLegCOMToHip);

	bool  updateModelNoMBS(int index, Eigen::Vector3d worldToPelvis_pos, Eigen::VectorXd lFoot, Eigen::VectorXd rFoot);

	//! Modifies pelvis position until the global CoM gets to the given point. 
	/*! 
	Remember that initial configuration must be feasible (i.e. solvable for IK).
		@param[in] initPelvisPos - initial pelvis position vector in world CF
		@param[in] globCoMPosRef - desire global CoM position vector in world CF
		@param[in] lFoot - left foot position and orientation vector in world CF
		@param[in] rFoot - right foot position and orientation vector in world CF
		@param[out] pelvisOut - final pelvis position vector.  
	*/
	bool get_pelvisPos_given_CoMPos(Eigen::Vector3d initPelvisPos, Eigen::Vector3d globCoMPosRef, Eigen::VectorXd lFoot, Eigen::VectorXd rFoot, Eigen::Vector3d *pelvisOut);
	
	//! Updates kinematic and dynamic state of the model in selected sample. Update samplse in order 0, 1, 2 ...
	/*! 
		@param[in] index - sample to be updated. Numbering from 0. 
		@param[in] worldToPelvis_x - x coordinate of pelvis position in world CF
		@param[in] worldToPelvis_y - y coordinate of pelvis position in world CF
		@param[in] worldToPelvis_z - z coordinate of pelvis position in world CF
		@param[in] lFoot - left foot position and orientation vector
		@param[in] rFoot - right foot position and orientation vector
	*/
	bool  updateModel(int index, double worldToPelvis_x, double worldToPelvis_y, double worldToPelvis_z, Eigen::VectorXd lFoot, Eigen::VectorXd rFoot);
	
	//! Updates kinematic and dynamic state of the model in selected sample. Update samples only in order 0, 1, 2 ...
	/*! 
		@param[in] index - sample to be updated. Numbering from 0
		@param[in] worldToPelvis_pos - pelvis position vector in world CF
		@param[in] lFoot - left foot position and orientation vector in world CF
		@param[in] rFoot - right foot position and orientation vector in world CF
	*/
	bool  updateModel(int index, Eigen::Vector3d worldToPelvis_pos, Eigen::VectorXd lFoot, Eigen::VectorXd rFoot);
	bool  updateModel_Harry(int index, Eigen::Vector3d worldToPelvis_pos, Eigen::VectorXd lFoot, Eigen::VectorXd rFoot, Eigen::Vector3d *leftLegCOMToHip, Eigen::Vector3d *rightLegCOMToHip);
	

	//! 
	/*!
		@param i - total number of samples. Be careful to not exceed it later when using updateModel()
		@param stepTime - step time used in calculation of velocities and accelerations. Units [s]
 	*/
	bool init(int i, double stepTime);
	
	//! Check if the instance was initialized.
	/*!
		@return true - if instance was alread initialized and false if not
	*/
	bool isInit();
	
	//! Copy zmp of a chosen point untill the end of trajectory. Used in preview controler
	/*!
		@param startingPoint - point which should be copied. Indexing from 0
	*/
	bool replicateZMP(int startingPoint);
	


// ************************* VARIABLES **********************************

	/// Links' CoM position. Each cell of the vector contains matrix of which rows correspond to x, y, z and columns correspond to mass point. 
	std::vector< Eigen::MatrixXd, Eigen::aligned_allocator< Eigen::MatrixXd > > m_indivCoM_pos;
	/// Links' CoM velocity. Each cell of the vector contains matrix of which rows correspond to x, y, z and columns correspond to mass point. 
	std::vector< Eigen::MatrixXd, Eigen::aligned_allocator< Eigen::MatrixXd > > m_indivCoM_vel;
	/// Links' CoM acceleration. Each cell of the vector contains matrix of which rows correspond to x, y, z and columns correspond to mass point. 
	std::vector< Eigen::MatrixXd, Eigen::aligned_allocator< Eigen::MatrixXd > > m_indivCoM_acc;

	
	/// Global CoM position. Each matrix rows correspond to x, y, z and columns to the sample in the current trajectory.   
	Eigen::MatrixXd m_globCoM_pos;
	/// Global CoM velocty. Each matrix rows correspond to x, y, z and columns to the sample in the current trajectory.   
	Eigen::MatrixXd m_globCoM_vel; 
	/// Global CoM acceleration. Each matrix rows correspond to x, y, z and columns to the sample in the current trajectory.   
	Eigen::MatrixXd m_globCoM_acc;

	/// Global ZMP trajectory resulting from MBS (Multi Body System). Each matrix rows correspond to x, y, z and columns to the sample in the current trajectory.   
	Eigen::MatrixXd zmp_multiBody;

	/// Pelvis position. Each cell contains information about orientation of pelvis in the particular sample of the trajectory. 
	std::vector< Eigen::MatrixXd, Eigen::aligned_allocator< Eigen::Matrix3d > > m_pelvis_rot;
	/// Pelvis orientation. Each cell contains information about position of pelvis in the particular sample of the trajectory. 
	std::vector< Eigen::MatrixXd, Eigen::aligned_allocator< Eigen::Vector3d > > m_pelvis_pos;
	

	///  Matrix containing joint space configuration in time. Rows -> joints, columns -> samples. 
	Eigen::MatrixXd m_q;

};// End of class "kinAndDyn"


#endif
