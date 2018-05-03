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
#include <ct/models/HyQ/HyQ.h>

#include <dynamic_reconfigure/server.h>
#include <ct_ros_nodes/SimulationConfig.h>


typedef ct::rbd::FloatingBaseFDSystem<ct::rbd::HyQ::Dynamics, false> HyQSystem;
typedef ct::rbd::ProjectedFDSystem<ct::rbd::HyQ::Dynamics, false> HyQSystemProjected;

typedef ct::rbd::EEContactModel<typename HyQSystem::Kinematics> ContactModel;

ct::rbd::RBDState<12> initRbdState;
std::shared_ptr<ContactModel> contactModel;
std::shared_ptr<ct::ros::RBDSimulationNode<HyQSystem>> simulator;
std::shared_ptr<ct::ros::RBDSimulationNode<HyQSystemProjected>> simulatorProjected;
std::shared_ptr<ct::rbd::WholeBodyController<12>> controller;
bool useProjectedDynamics = false;


void callback(ct_ros_nodes::SimulationConfig &config, uint32_t level) {

	initRbdState.basePose().position().toImplementation() << config.x_init, config.y_init, config.z_init;

	kindr::EulerAnglesXyzD eulerAngles(config.roll, config.pitch, config.yaw);
	initRbdState.basePose().setFromEulerAnglesXyz(eulerAngles);

	if(config.reset)
	{
		simulator->setState(initRbdState);
		simulatorProjected->setState(initRbdState);
		controller->getJointController().reset();
	}

	config.reset = false;


	contactModel->k() = config.k;
	contactModel->d() = config.d;
	contactModel->alpha() = config.alpha;
	contactModel->alpha_n() = config.alpha_n;
	contactModel->zOffset() = config.zOffset;

	contactModel->smoothing() = static_cast<typename ContactModel::VELOCITY_SMOOTHING>(config.smoothing);
}


int main(int argc, char* argv[])
{
	std::cout << "Initializing HyQ Simulator" << std::endl;

	std::cout << "Setting up ROS" << std::endl;

	ros::init(argc, argv, "hyq_simulator");
	ros::NodeHandle nh("~");

	nh.param("useProjectedDynamics", useProjectedDynamics, false);

	std::cout << "Setting up controller" << std::endl;
	controller = std::shared_ptr<ct::rbd::WholeBodyController<12>>(new ct::rbd::WholeBodyController<12>);
	controller->getJointController().setAllPIDGains(600.0, 0.0, 6.0);
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
	controller->getJointController().setDesiredPosition(desiredPos);

	std::cout << "Setting up system" << std::endl;
	std::shared_ptr<HyQSystem> system(new HyQSystem);
	system->setController(controller);
	contactModel = std::shared_ptr<ContactModel>(new ContactModel);
	system->setContactModel(contactModel);

	std::cout << "Setting up projected system" << std::endl;
	std::shared_ptr<HyQSystemProjected> systemProjected(new HyQSystemProjected);
	systemProjected->setController(controller);

	std::cout << "Setting up publisher" << std::endl;

	std::shared_ptr<ct::ros::RBDStatePublisher> statePublisher(new ct::ros::RBDStatePublisher(ct::models::HyQ::urdfJointNames(), "/base_link", "/world"));
	statePublisher->advertise(nh, "/joint_states", 10);

	std::cout << "Setting up simulator node" << std::endl;
	simulator = std::shared_ptr<ct::ros::RBDSimulationNode<HyQSystem>>(new ct::ros::RBDSimulationNode<HyQSystem>(statePublisher, system));
	// simulatorProjected = std::shared_ptr<ct::ros::RBDSimulationNode<HyQSystemProjected>>(new ct::ros::RBDSimulationNode<HyQSystemProjected>(statePublisher, systemProjected));

	dynamic_reconfigure::Server<ct_ros_nodes::SimulationConfig> server;
	dynamic_reconfigure::Server<ct_ros_nodes::SimulationConfig>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	initRbdState.setZero();
	initRbdState.basePose().position()(2) = 1.0;
	initRbdState.jointPositions() = desiredPos*1.5;
	simulator->setState(initRbdState);
	simulatorProjected->setState(initRbdState);

	std::cout << "Initializiation complete. Will now run." << std::endl;
	if (useProjectedDynamics)
	{
		simulatorProjected->simulate();
	}
	else
	{
		simulator->simulate();
	}

	std::cout << "HyQ Simulator interrupted. Exiting." << std::endl;

	return 1;
}
