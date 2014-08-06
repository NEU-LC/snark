#include "data.h"
#include <iostream>

namespace snark { namespace ur { namespace robotic_arm { 

const char* robotmode_str( robotmode::mode mode )
{
    switch( mode )
    {
        case robotmode::running:
            return "RN"; 
        case robotmode::freedrive:
            return "FD";
        case robotmode::ready:
            return "RD";
        case robotmode::initializing:
            return "IN";
        case robotmode::security_stopped:
            return "SS";
        case robotmode::estopped:
            return "ES";
        case robotmode::fatal_error:
            return "FE";
        case robotmode::no_power:
            return "NP";
        case robotmode::not_connected:
            return "NC";
        case robotmode::shutdown:
            return "SH";
        case robotmode::safeguard_stop:
            return "SG";
        default:
            std::cerr << "unknown robot mode: " << int(mode) << std::endl;
            COMMA_THROW( comma::exception, "unknown robot mode" );
            return "UN";
    }
}

robotmode::mode get_robotmode(const std::string& mode)
{
    if( mode == "RN" ) { return robotmode::running; }
    else if( mode == "FD" ) { return robotmode::freedrive; }
    else if( mode == "RD" ) { return robotmode::ready; }
    else if( mode == "IN" ) { return robotmode::initializing; }
    else if( mode == "SS" ) { return robotmode::security_stopped; }
    else if( mode == "FE" ) { return robotmode::fatal_error; }
    else if( mode == "NP" ) { return robotmode::no_power; }
    else if( mode == "NC" ) { return robotmode::not_connected; }
    else if( mode == "SH" ) { return robotmode::shutdown; }
    else if( mode == "SG" ) { return robotmode::safeguard_stop; }
    else { COMMA_THROW( comma::exception, "unknown robot mode given: " << mode ); }

}

jointmode::mode get_jointmode(const std::string& mode)
{
    if( mode == "PO" ) { return jointmode::power_off; }
    else if( mode == "ER" ) { return jointmode::error; }
    else if( mode == "FD" ) { return jointmode::freedrive; }
    else if( mode == "CL" ) { return jointmode::calibration; }
    else if( mode == "SS" ) { return jointmode::stopped; }
    else if( mode == "RN" ) { return jointmode::running; }
    else if( mode == "IN" ) { return jointmode::initializing; }
    else if( mode == "ID" ) { return jointmode::idle; }
    else if( mode == "OT" ) { return jointmode::other; }
    else { COMMA_THROW( comma::exception, "unknown joint mode given: " << mode ); }
}



const char* jointmode_str( jointmode::mode mode )
{
    switch( mode )
    {
        case jointmode::power_off:
            return "PO";
        case jointmode::error:
            return "ER";
        case jointmode::freedrive:
            return "FD";
        case jointmode::calibration:
            return "CL";
        case jointmode::stopped:
            return "SS";
        case jointmode::running:
            return "RN";
        case jointmode::initializing:
            return "IN";
        case jointmode::idle:
            return "ID";
        default:
            return "OT"; // some other mode not converted to string
    }
}

void fixed_status::get_angles(boost::array< plane_angle_t, 6 >& angles)
{
    angles[0] = this->positions[0]() * radian;
    angles[1] = this->positions[1]() * radian;
    angles[2] = this->positions[2]() * radian;
    angles[3] = this->positions[3]() * radian;
    angles[4] = this->positions[4]() * radian;
    angles[5] = this->positions[5]() * radian;
}

void set_array( const joints_net_t& values, status_t::array_doubles_t& arr )
{
    arr[0] = values[0]();
    arr[1] = values[1]();
    arr[2] = values[2]();
    arr[3] = values[3]();
    arr[4] = values[4]();
    arr[5] = values[5]();
}

void fixed_status::get(status_t& st)
{
    st.position.coordinates = Eigen::Vector3d( this->translation.x(), this->translation.y(), this->translation.z() );
    st.position.orientation = Eigen::Vector3d( this->rotation.x(), this->rotation.y(), this->rotation.z() );
    this->get_angles( st.joint_angles );
    set_array( velocities, st.velocities );
    set_array( currents, st.currents );
    set_array( forces, st.forces );
    set_array( temperatures, st.temperatures );
    st.robot_mode = robotmode::mode( int(this->robot_mode()) );
    st.joint_modes[0] = jointmode::mode( int( this->joint_modes[0]() ) );
    st.joint_modes[1] = jointmode::mode( int( this->joint_modes[1]() ) );
    st.joint_modes[2] = jointmode::mode( int( this->joint_modes[2]() ) );
    st.joint_modes[3] = jointmode::mode( int( this->joint_modes[3]() ) );
    st.joint_modes[4] = jointmode::mode( int( this->joint_modes[4]() ) );
    st.joint_modes[5] = jointmode::mode( int( this->joint_modes[5]() ) );
    st.length = length();
    st.time_since_boot = time_since_boot();
}

bool status_t::is_running() const
{
    if( robot_mode != robotmode::running) { 
        // std::cerr << "robot mode " << robot_mode << " expected: " << robotmode::running << std::endl;
        return false; 
    }

    for( std::size_t i=0; i<joints_num; ++i ) {
        // std::cerr << "joint " << i << " mode " << joint_modes[i] << " expected: " << jointmode::running << std::endl;
        if( joint_modes[i] != jointmode::running ) { return false; }
    }
    
    return true;
}



} } } // namespace snark { namespace ur { namespace robotic_arm { 
