//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Your name, institution, or company name.
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================


//------------------------------------------------------------------------------
#include "system/CGlobals.h"
#include "devices/SpaceMouse.h"
#include <iostream>
#include <cmath>
//------------------------------------------------------------------------------
#if defined(C_ENABLE_SPACEMOUSE_DEVICE_SUPPORT)
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
/*
    INSTRUCTION TO IMPLEMENT YOUR OWN CUSTOM DEVICE:

    Please review header file SpaceMouse.h for some initial 
    guidelines about how to implement your own haptic device using this
    template.

    When ready, simply completed the next 11 documented steps described here
    below.
*/
////////////////////////////////////////////////////////////////////////////////

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------



//==============================================================================
/*!
    Constructor of SpaceMouse.
*/
//==============================================================================
SpaceMouse::SpaceMouse(unsigned int a_deviceNumber)
{
    // the connection to your device has not yet been established.
    m_deviceReady = false;
    m_deviceAvailable = false;
    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 1:

        Here you should define the specifications of your device.
        These values only need to be estimates. Since haptic devices often perform
        differently depending of their configuration withing their workspace,
        simply use average values.
    */
    ////////////////////////////////////////////////////////////////////////////

    //--------------------------------------------------------------------------
    // NAME:
    //--------------------------------------------------------------------------

    // haptic device model (see file "CGenericHapticDevice.h")
    m_specifications.m_model                         = C_SPACEMOUSE;

    // name of the device manufacturer, research lab, university.
    m_specifications.m_manufacturerName              = "3DConnexion";

    // name of your device
    m_specifications.m_modelName                     = "SpaceMouse";


    //--------------------------------------------------------------------------
    // CHARACTERISTICS: (The following values must be positive or equal to zero)
    //--------------------------------------------------------------------------

    // the maximum force [N] the device can produce along the x,y,z axis.
    m_specifications.m_maxLinearForce                = 0.0;     // [N]

    // the maximum amount of torque your device can provide arround its
    // rotation degrees of freedom.
    m_specifications.m_maxAngularTorque              = 0.0;     // [N*m]

    // the maximum amount of torque which can be provided by your gripper
    m_specifications.m_maxGripperForce                = 0.0;     // [N]

    // the maximum closed loop linear stiffness in [N/m] along the x,y,z axis
    m_specifications.m_maxLinearStiffness             = 0.0; // [N/m]

    // the maximum amount of angular stiffness
    m_specifications.m_maxAngularStiffness            = 0.0;    // [N*m/Rad]

    // the maximum amount of stiffness supported by the gripper
    m_specifications.m_maxGripperLinearStiffness      = 0.0;   // [N*m]

    // the radius of the physical workspace of the device (x,y,z axis)
    m_specifications.m_workspaceRadius                = 1.0;     // [m]

    // the maximum opening angle of the gripper
    m_specifications.m_gripperMaxAngleRad             = cDegToRad(0.0);


    ////////////////////////////////////////////////////////////////////////////
    /*
        DAMPING PROPERTIES:

        Start with small values as damping terms can be high;y sensitive to 
        the quality of your velocity signal and the spatial resolution of your
        device. Try gradually increasing the values by using example "01-devices" 
        and by enabling viscosity with key command "2".
    */
    ////////////////////////////////////////////////////////////////////////////
    
    // Maximum recommended linear damping factor Kv
    m_specifications.m_maxLinearDamping             = 20.0;   // [N/(m/s)]

    //! Maximum recommended angular damping factor Kv (if actuated torques are available)
    m_specifications.m_maxAngularDamping            = 0.0;    // [N*m/(Rad/s)]

    //! Maximum recommended angular damping factor Kv for the force gripper. (if actuated gripper is available)
    m_specifications.m_maxGripperAngularDamping     = 0.0;    // [N*m/(Rad/s)]


    //--------------------------------------------------------------------------
    // CHARACTERISTICS: (The following are of boolean type: (true or false)
    //--------------------------------------------------------------------------

    // does your device provide sensed position (x,y,z axis)?
    m_specifications.m_sensedPosition                = true;

    // does your device provide sensed rotations (i.e stylus)?
    m_specifications.m_sensedRotation                = true;

    // does your device provide a gripper which can be sensed?
    m_specifications.m_sensedGripper                 = true;

    // is you device actuated on the translation degrees of freedom?
    m_specifications.m_actuatedPosition              = false;

    // is your device actuated on the rotation degrees of freedom?
    m_specifications.m_actuatedRotation              = false;

    // is the gripper of your device actuated?
    m_specifications.m_actuatedGripper               = false;

    // can the device be used with the left hand?
    m_specifications.m_leftHand                      = true;

    // can the device be used with the right hand?
    m_specifications.m_rightHand                     = true;


    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 2:

        Here, you shall  implement code which tells the application if your
        device is actually connected to your computer and can be accessed.
        In practice this may be consist in checking if your I/O board
        is active or if your drivers are available.

        If your device can be accessed, set:
        m_systemAvailable = true;

        Otherwise set:
        m_systemAvailable = false;

        Your actual code may look like:

        bool result = checkIfMyDeviceIsAvailable()
        m_systemAvailable = result;

        If want to support multiple devices, using the method argument
        a_deviceNumber to know which device to setup
    */  
        
    m_deviceAvailable = (getNumDevices() > a_deviceNumber);

    haptic_twist_state_.push_back(0.0); // vx
    haptic_twist_state_.push_back(0.0); // vy
    haptic_twist_state_.push_back(0.0); // vz
    haptic_twist_state_.push_back(0.0); // vroll
    haptic_twist_state_.push_back(0.0); // vpitch
    haptic_twist_state_.push_back(0.0); // vyaw
}


//==============================================================================
/*!
    Destructor of SpaceMouse.
*/
//==============================================================================
SpaceMouse::~SpaceMouse()
{
    // close connection to device
    if (m_deviceReady)
    {
        close();
    }
   
}


//==============================================================================
/*!
    This method opens a connection to your device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool SpaceMouse::open()
{
    // check if the system is available
    if (!m_deviceAvailable) return (C_ERROR);

    // if system is already opened then return
    if (m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 3:

        Here you shall implement to open a connection to your
        device. This may include opening a connection to an interface board
        for instance or a USB port.

        If the connection succeeds, set the variable 'result' to true.
        otherwise, set the variable 'result' to false.

        Verify that your device is calibrated. If your device 
        needs calibration then call method calibrate() for wich you will 
        provide code in STEP 5 further below.
    */
    ////////////////////////////////////////////////////////////////////////////


    if (spnav_open() == -1) {
           std::cout << "Could not open the space navigator device. "
            "Did you remember to run spacenavd (as root)?" << std::endl;
        m_deviceReady = false;
        return false ;
    } 

    m_deviceReady = true;
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method closes the connection to your device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool SpaceMouse::close()
{
    // check if the system has been opened previously
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 4:

        Here you shall implement code that closes the connection to your
        device.

        If the operation fails, simply set the variable 'result' to C_ERROR   .
        If the connection succeeds, set the variable 'result' to C_SUCCESS.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS; // if the operation fails, set value to C_ERROR.

    int int_result = spnav_close();

    if (int_result == -1) {
        result = C_ERROR;
    }

    // update status
    m_deviceReady = false;

    return (result);
}


//==============================================================================
/*!
    This method calibrates your device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool SpaceMouse::calibrate(bool a_forceCalibration)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 5:
        
        Here you shall implement code that handles a calibration procedure of the 
        device. In practice this may include initializing the registers of the
        encoder counters for instance. 

        If the device is already calibrated and  a_forceCalibration == false,
        the method may immediately return without further action.
        If a_forceCalibration == true, then the calibrartion procedure
        shall be executed even if the device has already been calibrated.
 
        If the calibration procedure succeeds, the method returns C_SUCCESS,
        otherwise return C_ERROR.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // *** INSERT YOUR CODE HERE ***

    // error = calibrateMyDevice()

    return (result);
}


//==============================================================================
/*!
    This method returns the number of devices available from this class of device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
unsigned int SpaceMouse::getNumDevices()
{
    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 6:

        Here you shall implement code that returns the number of available
        haptic devices of type "SpaceMouse" which are currently connected
        to your computer.

        In practice you will often have either 0 or 1 device. In which case
        the code here below is already implemented for you.

        If you have support more than 1 devices connected at the same time,
        then simply modify the code accordingly so that "numberOfDevices" takes
        the correct value.
    */
    ////////////////////////////////////////////////////////////////////////////

    if (spnav_open() == -1) {
        return 0;
    }

    spnav_close();
    return 1;
}


//==============================================================================
/*!
    This method returns the position of your device. Units are meters [m].

    \param   a_position  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool SpaceMouse::getPosition(cVector3d& a_position)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 7:

        Here you shall implement code that reads the position (X,Y,Z) from
        your haptic device. Read the values from your device and modify
        the local variable (x,y,z) accordingly.
        If the operation fails return an C_ERROR, C_SUCCESS otherwise

        Note:
        For consistency, units must be in meters.
        If your device is located in front of you, the x-axis is pointing
        towards you (the operator). The y-axis points towards your right
        hand side and the z-axis points up towards the sky. 
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // *** INSERT YOUR CODE HERE, MODIFY CODE below ACCORDINGLY ***

    switch(spnav_poll_event(&sev)) {
        case 0:
            if (++no_motion_count > static_count_threshold) {
                haptic_twist_state_[0] = 0.0;
                haptic_twist_state_[1] = 0.0;
                haptic_twist_state_[2] = 0.0;
                haptic_twist_state_[3] = 0.0;
                haptic_twist_state_[4] = 0.0;
                haptic_twist_state_[5] = 0.0;
                no_motion_count = 0; // reset counter after zeroing velocities
            }
            break;
        
        case SPNAV_EVENT_MOTION:
            haptic_twist_state_[0] = sev.motion.z / full_scale;
            haptic_twist_state_[1] = -sev.motion.x / full_scale;
            haptic_twist_state_[2] = sev.motion.y / full_scale;
            haptic_twist_state_[3] = sev.motion.rz / full_scale; //roll
            haptic_twist_state_[4] = -sev.motion.rx / full_scale; //pitch
            haptic_twist_state_[5] = sev.motion.ry / full_scale; //yaw
            break;
        case SPNAV_EVENT_BUTTON:
            if (sev.button.bnum < static_cast<int>(buttons.size())) {
                // Update known buttons
                buttons[sev.button.bnum] = sev.button.press;
            } else {
                // Enlarge, fill up with zeros, and support the new button
                buttons.resize(sev.button.bnum + 1, false);
                buttons[sev.button.bnum] = sev.button.press;
            }
            break;
        default:
            break;
    }
    // estimate linear velocity
    // estimateLinearVelocity(a_position);
    m_linearVelocity.zero();
    m_linearVelocity(0) = haptic_twist_state_[0];
    m_linearVelocity(1) = haptic_twist_state_[1];
    m_linearVelocity(2) = haptic_twist_state_[2];

    //update angular velocity
    m_angularVelocity.zero();
    m_angularVelocity(0) = haptic_twist_state_[3];
    m_angularVelocity(1) = haptic_twist_state_[4];
    m_angularVelocity(2) = haptic_twist_state_[5];

    // store new position values
    a_position.set(0.0, 0.0, 0.0);

    // exit
    return (result);
}


//==============================================================================
/*!
    This method returns the orientation frame of your device end-effector

    \param   a_rotation  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool SpaceMouse::getRotation(cMatrix3d& a_rotation)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 8:

        Here you shall implement code which reads the orientation frame from
        your haptic device. The orientation frame is expressed by a 3x3
        rotation matrix. The 1st column of this matrix corresponds to the
        x-axis, the 2nd column to the y-axis and the 3rd column to the z-axis.
        The length of each column vector should be of length 1 and vectors need
        to be orthogonal to each other.

        Note:
        If your device is located in front of you, the x-axis is pointing
        towards you (the operator). The y-axis points towards your right
        hand side and the z-axis points up towards the sky.

        If your device has a stylus, make sure that you set the reference frame
        so that the x-axis corresponds to the axis of the stylus.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // double roll = 0.0; //roll
    // double pitch = 0.0; //pitch
    // double yaw = 0.0; //yaw



    // variables that describe the rotation matrix
    double r00, r01, r02, r10, r11, r12, r20, r21, r22;
    cMatrix3d frame;
    frame.identity();

    // *** INSERT YOUR CODE HERE, MODIFY CODE below ACCORDINGLY ***

    // if the device does not provide any rotation capabilities 
    // set the rotation matrix equal to the identity matrix.
    // r00 = cos(pitch)*cos(yaw);
    // r01 = sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);
    // r02 = cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
    // r10 = cos(pitch)*sin(yaw);
    // r11 = sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw);
    // r12 = cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);
    // r20 = -sin(pitch);
    // r21 = sin(roll)*cos(pitch);
    // r22 = cos(roll)*cos(pitch);

    frame.set(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

    // store new rotation matrix
    a_rotation = frame;

    // estimate angular velocity
    // estimateAngularVelocity(a_rotation);

    // m_angularVelocity.zero();
    // m_angularVelocity(0) = roll;
    // m_angularVelocity(1) = pitch;
    // m_angularVelocity(2) = yaw;

    // exit
    return (result);
}


//==============================================================================
/*!
    This method returns the gripper angle in radian.

    \param   a_angle  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool SpaceMouse::getGripperAngleRad(double& a_angle)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 9:
        Here you may implement code which reads the position angle of your
        gripper. The result must be returned in radian.

        If the operation fails return an error code such as C_ERROR for instance.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // *** INSERT YOUR CODE HERE, MODIFY CODE below ACCORDINGLY ***

    // return gripper angle in radian
    a_angle = 0.0;  // a_angle = getGripperAngleInRadianFromMyDevice();

    // estimate gripper velocity
    estimateGripperVelocity(a_angle);

    // exit
    return (result);
}


//==============================================================================
/*!
    This method sends a force [N] and a torque [N*m] and gripper torque [N*m] 
    to your haptic device.

    \param   a_force  Force command.
    \param   a_torque  Torque command.
    \param   a_gripperForce  Gripper force command.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool SpaceMouse::setForceAndTorqueAndGripperForce(const cVector3d& a_force,
                                                       const cVector3d& a_torque,
                                                       const double a_gripperForce)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 10:
        
        Here you may implement code which sends a force (fx,fy,fz),
        torque (tx, ty, tz) and/or gripper force (gf) command to your haptic device.

        If your device does not support one of more of the force, torque and 
        gripper force capabilities, you can simply ignore them. 

        Note:
        For consistency, units must be in Newtons and Newton-meters
        If your device is placed in front of you, the x-axis is pointing
        towards you (the operator). The y-axis points towards your right
        hand side and the z-axis points up towards the sky.

        For instance: if the force = (1,0,0), the device should move towards
        the operator, if the force = (0,0,1), the device should move upwards.
        A torque (1,0,0) would rotate the handle counter clock-wise around the 
        x-axis.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // store new force value.
    m_prevForce = a_force;
    m_prevTorque = a_torque;
    m_prevGripperForce = a_gripperForce;

    // retrieve force, torque, and gripper force components in individual variables
    double fx = a_force(0);
    double fy = a_force(1);
    double fz = a_force(2);

    double tx = a_torque(0);
    double ty = a_torque(1);
    double tz = a_torque(2);

    double gf = a_gripperForce;

    // *** INSERT YOUR CODE HERE ***

    // setForceToMyDevice(fx, fy, fz);
    // setTorqueToMyDevice(tx, ty, tz);
    // setForceToGripper(fg);


    // exit
    return (result);
}


//==============================================================================
/*!
    This method returns status of all user switches 
    [__true__ = __ON__ / __false__ = __OFF__].

    \param  a_userSwitches  Return the 32-bit binary mask of the device buttons.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool SpaceMouse::getUserSwitches(unsigned int& a_userSwitches)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 11:

        Here you shall implement code that reads the status all user switches 
        on your device. For each user switch, set the associated bit on variable
        a_userSwitches. If your device only has one user switch, then set 
        a_userSwitches to 1, when the user switch is engaged, and 0 otherwise.
    */
    ////////////////////////////////////////////////////////////////////////////

    a_userSwitches = 0;
    for(int i = 0; i < static_cast<int>(buttons.size()); ++i) {
        if (buttons[i]) {
            a_userSwitches |= (1 << i);
        }
    }
    return (C_SUCCESS);
}


//------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------
#endif  // C_ENABLE_SPACEMOUSE_DEVICE_SUPPORT
//------------------------------------------------------------------------------
