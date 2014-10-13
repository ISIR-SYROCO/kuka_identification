// Filename:  kukaIdentification-rtnetcomponent.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#include "kukaIdentification-rtnetcomponent.hpp"
#include <cmath>
#include <fstream>
#include <iostream>

#include <rtt/Component.hpp>

KukaIdentificationRTNET::KukaIdentificationRTNET(std::string const& name) : FriRTNetExampleAbstract(name){
    t.assign(LWRDOF, 0);

    goToZero = true;
    loging = false;
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

    position_log.reserve(20000);
    gravity_log.reserve(20000);
    mass_matrix_log.reserve(20000);
}

void KukaIdentificationRTNET::computeJointPosition(){
    if(goToZero == true){
        if(current_pos.norm() < 0.001){
            goToZero = false;
            loging = true;
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
        for(unsigned int i=0; i<LWRDOF; ++i){
            if(phi[i] <= omega[i]*0.0001*t[0]){
                joint_pos_command[i] = qlimit[i] * sin(omega[i]*0.0001*t[i]);
                t[i]++;
            }
        }
    }
}

void KukaIdentificationRTNET::dumpLog(){
    std::ofstream file;
    file.open("log.txt");
    for(unsigned int i=0; i<position_log.size(); ++i){
        file << position_log[i].transpose() << std::endl << std::endl;

        file << mass_matrix_log[i] << std::endl << std::endl;

        file << gravity_log[i].transpose() << std::endl << std::endl;
    }
    file.close();
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
        if(loging){
            position_log.push_back(current_pos);
        }
    }


    RTT::FlowStatus mass_matrix_fs = iport_mass_matrix.read(mass_matrix);
    if(mass_matrix_fs == RTT::NewData && loging){
        mass_matrix_log.push_back(mass_matrix);
    }

    std::vector<double> gravity_(LWRDOF);
    RTT::FlowStatus gravity_fs = gravityPort.read(gravity_);
    if(gravity_fs == RTT::NewData && loging){
        for(unsigned int i = 0; i < LWRDOF; i++){
            gravity[i] = gravity_[i];
        }
        gravity_log.push_back(gravity);
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

void KukaIdentificationRTNET::stopHook(){
    dumpLog();
}

ORO_CREATE_COMPONENT(KukaIdentificationRTNET)
