/***********************************************************************************
Copyright (c) 2017, Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo,
Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be used
      to endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/


#include <ct/rbd/rbd.h>

#include <ct/ros/ros.h>

#include <ct/models/HyQ/HyQUrdfNames.h>


int main(int argc, char* argv[])
{
	std::cout << "Initializing HyQ Visualization" << std::endl;

	std::cout << "Setting up ROS" << std::endl;

	ros::init(argc, argv, "hyq_simulator");
	ros::NodeHandle nh;

	std::cout << "Setting up publisher" << std::endl;

	std::shared_ptr<ct::ros::RBDStatePublisher> statePublisher(new ct::ros::RBDStatePublisher(ct::models::HyQ::urdfJointNames(), "/base_link", "/world"));
	statePublisher->advertise(nh, "/joint_states", 10);


	ct::rbd::RBDState<12> rbdState;
	rbdState.setZero();
	Eigen::Matrix<double, 12, 1> desiredPos;
	desiredPos <<
			-0.2  ,
			 0.723,
			-1.458,
			-0.2  ,
			 0.723,
			-1.458,
			-0.2  ,
			-0.723,
			 1.458,
			-0.2  ,
			-0.723,
			 1.458;
	rbdState.jointPositions() = desiredPos;

	enum SEQUENCE
	{
		MOVE_X,
		MOVE_Y,
		MOVE_Z,
		ROLL,
		PITCH,
		YAW
	};

	double posMax = 1.0;
	double angleMax = 45./180.*M_PI;
	int subSteps = 100;

	SEQUENCE sequence = MOVE_X;
	int seqCounter = 0;

	ros::Rate publishRate(subSteps);

	while(ros::ok())
	{
		switch(sequence)
		{
		case MOVE_X:
		{
			rbdState.basePose().position().x() += posMax / subSteps;
			break;
		}
		case MOVE_Y:
		{
			rbdState.basePose().position().y() += posMax / subSteps;
			break;
		}
		case MOVE_Z:
		{
			rbdState.basePose().position().z() += posMax / subSteps;
			break;
		}

		case ROLL:
		{
			kindr::EulerAnglesXyzD angles  = rbdState.basePose().getEulerAnglesXyz();
			angles.setX(angles.x() + angleMax/subSteps);
			rbdState.basePose().setFromEulerAnglesXyz(angles);
			break;
		}
		case PITCH:
		{
			kindr::EulerAnglesXyzD angles  = rbdState.basePose().getEulerAnglesXyz();
			angles.setY(angles.y() + angleMax/subSteps);
			rbdState.basePose().setFromEulerAnglesXyz(angles);
			break;
		}
		case YAW:
		{
			kindr::EulerAnglesXyzD angles  = rbdState.basePose().getEulerAnglesXyz();
			angles.setZ(angles.z() + angleMax/subSteps);
			rbdState.basePose().setFromEulerAnglesXyz(angles);
			break;
		}

		default:
			std::cout << "illegal case number" << std::endl;
			exit(-1);
		}

		statePublisher->publishState(rbdState);

		seqCounter++;

		if (seqCounter >= subSteps)
		{
			seqCounter = 0;
			sequence = static_cast<SEQUENCE>(static_cast<int>(sequence) + 1);

			if (sequence > YAW)
			{
				sequence = MOVE_X;
				rbdState.setZero();
				rbdState.jointPositions() = desiredPos;
			}
		}

		publishRate.sleep();
	}

	std::cout << "HyQ Visualization interrupted. Exiting." << std::endl;

	return 1;
}
