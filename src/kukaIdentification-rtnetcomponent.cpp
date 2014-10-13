// Filename:  kukaIdentification-rtnetcomponent.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#include "kukaIdentification-rtnetcomponent.hpp"
#include <cmath>

#include <rtt/Component.hpp>

KukaIdentificationRTNET::KukaIdentificationRTNET(std::string const& name) : FriRTNetExampleAbstract(name){
    t.assign(LWRDOF, 0);

    goToZero = true;
    v = 0.0;

    qlimit.reserve(LWRDOF);
    qlimit[0] = 2.87;
    qlimit[1] = 2.00;
    qlimit[2] = 2.87;
    qlimit[3] = 2.00;
    qlimit[4] = 2.87;
    qlimit[5] = 2.00;
    qlimit[6] = 2.87;

    omega.reserve(LWRDOF);
    omega[0] = 0.7;
    omega[1] = 0.4;
    omega[2] = 0.3;
    omega[3] = 0.2;
    omega[4] = 0.3;
    omega[5] = 0.4;
    omega[6] = 0.5;

    phi.reserve(LWRDOF);
    phi[0] = 0.0;
    phi[1] = 0.1;
    phi[2] = 0.3;
    phi[3] = 0.5;
    phi[4] = 0.7;
    phi[5] = 1.1;
    phi[6] = 1.3;

	joint_pos_command.assign(LWRDOF, 0.0);
	gravity.assign(LWRDOF, 0.0);
}

void KukaIdentificationRTNET::computeJointPosition(){
    if(goToZero == true){
        if(current_pos.norm() < 0.001){
            goToZero = false;
        }
        for(unsigned int i=0; i<LWRDOF; ++i){
            if(current_pos[i] < -0.2)
                joint_pos_command[i] += v*getPeriod();
            else if(current_pos[i] > 0.2)
                joint_pos_command[i] -= v*getPeriod();
            else
                joint_pos_command[i] -= current_pos[i]*getPeriod();
	    if(v<0.2)
	        v+=0.00001;
        }
    }
    else{
        for(unsigned int i=0; i<2; ++i){
            if(phi[i] <= omega[i]*0.0001*t[i]){
                joint_pos_command[i] = qlimit[i] * sin(omega[i]*0.0001*t[i]);
                t[i]++;
            }
        }
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
        for(unsigned int i = 0; i < LWRDOF; i++){
            current_pos[i] = JState[i];
        }
    }


    RTT::FlowStatus mass_matrix_fs = iport_mass_matrix.read(mass_matrix);
    if(mass_matrix_fs == RTT::NewData){

    }

    RTT::FlowStatus gravity_fs = gravityPort.read(gravity);
    if(gravity_fs == RTT::NewData){

    }


    if (fri_cmd_mode){
        computeJointPosition();
        if(requiresControlMode(10)){
            oport_joint_position.write(joint_pos_command);
        }
    }
    else{
        for(unsigned int i = 0; i < LWRDOF; i++){
            joint_pos_command[i] = JState[i];
        }
    }
}

ORO_CREATE_COMPONENT(KukaIdentificationRTNET)
