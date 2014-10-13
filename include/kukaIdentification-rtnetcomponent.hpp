// Filename:  kukaIdentification-rtnetcomponent.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#ifndef KUKAIDENTIFICATION_RTNETCOMPONENT_HPP
#define KUKAIDENTIFICATION_RTNETCOMPONENT_HPP 

#include <friRTNetExampleAbstract.hpp>

class KukaIdentificationRTNET : public FriRTNetExampleAbstract{
    public:
        KukaIdentificationRTNET(std::string const& name);
        void updateHook();

        std::vector<double> joint_pos_command;
        Eigen::Matrix<double, 7, 1> current_pos;
        Eigen::Matrix<double, 7, 7> mass_matrix;
        std::vector<double> gravity;

        void computeJointPosition();

        std::vector<double> qlimit;

        std::vector<double> omega;
        std::vector<double> phi;

        std::vector<unsigned int> t;

        bool goToZero;
	double v;
};

#endif /* KUKAIDENTIFICATION-RTNETCOMPONENT_HPP */
