// Filename:  kukaIdentification-rtnetcomponent.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#include "kukaIdentification-rtnetcomponent.hpp"
#include <cmath>

#include <rtt/Component.hpp>

KukaIdentificationRTNET::KukaIdentificationRTNET(std::string const& name) : FriRTNetExampleAbstract(name){
    t = 0;

    qlimit.reserve(LWRDOF);
    qlimit[0] = 2.97;
    qlimit[1] = 2.10;
    qlimit[2] = 2.97;
    qlimit[3] = 2.10;
    qlimit[4] = 2.97;
    qlimit[5] = 2.10;
    qlimit[6] = 2.97;

    omega.reserve(LWRDOF);
    omega[0] = 0.7;
    omega[1] = 0.4;
    omega[2] = 0.3;
    omega[3] = 0.2;
    omega[4] = 0.3;
    omega[5] = 0.4;
    omega[6] = 0.5;

    phi.reserve(LWRDOF);
    phi[0] = 0.1;
    phi[1] = 0.3;
    phi[2] = 0.5;
    phi[3] = 0.7;
    phi[4] = 1.1;
    phi[5] = 1.3;
    phi[6] = 1.7;

	joint_vel_command.assign(LWRDOF, 0.0);
	gravity.assign(LWRDOF, 0.0);
}

void KukaIdentificationRTNET::computeJointVelocity(unsigned int t){
    for(unsigned int i = 0; i<LWRDOF; ++i){
        joint_vel_command[i] = qlimit[i] * cos(omega[i]*t + phi[i]);
    }
}

void KukaIdentificationRTNET::updateHook(){
    std::string fri_mode("e_fri_unkown_mode");
    bool fri_cmd_mode = false;
    RTT::FlowStatus fs_event = iport_events.read(fri_mode);
    if (fri_mode == "e_fri_cmd_mode")
        fri_cmd_mode = true;
    else if (fri_mode == "e_fri_mon_mode")
        fri_cmd_mode = false;
        
    std::vector<double> JState(LWRDOF);
    RTT::FlowStatus joint_state_fs = iport_msr_joint_pos.read(JState);

    if(joint_state_fs == RTT::NewData){        
        Eigen::VectorXd joint_pos(LWRDOF);
        for(unsigned int i = 0; i < LWRDOF; i++){
            joint_pos[i] = JState[i];
        }
    }


    RTT::FlowStatus mass_matrix_fs = iport_mass_matrix.read(mass_matrix);
    if(mass_matrix_fs == RTT::NewData){

    }

    RTT::FlowStatus gravity_fs = gravityPort.read(gravity);
    if(gravity_fs == RTT::NewData){

    }

    computeJointVelocity(t);

    if (fri_cmd_mode){
        if(requiresControlMode(10)){
            oport_joint_velocities.write(joint_vel_command);
        }
    }

    t++;
}

ORO_CREATE_COMPONENT(KukaIdentificationRTNET)
