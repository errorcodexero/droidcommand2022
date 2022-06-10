package org.xero1425.base.motors ;

import java.util.List ;

import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import java.util.ArrayList ;

/// \file

/// \brief This class acts like a MotorController but in reality is a group of motors that are mechanically
/// coupled and setup such that all but the first motor follows the first motor.  For the most part, calls to this
/// object are referred to the first motor controller in the group.
public class MotorGroupController extends XeroMotorController
{ 
    // The set of motors that are grouped
    private List<XeroMotorController> motors_ ;

    /// \brief Create a new MotorGroupController
    /// \param name the name of the group
    public MotorGroupController(MessageLogger logger, String name) {
        super(logger, name) ;
        motors_ = new ArrayList<XeroMotorController>() ;
    }

    public void disable() {
        if (motors_.size() == 0) {
            getMessageLogger().startMessage(MessageType.Error).add("disabled() called on an empty MotorGroupController").endMessage();
            return ;
        }

        for(XeroMotorController ctrl : motors_)
            ctrl.disable();
    }

    public void stopMotor() {
        if (motors_.size() == 0) {
            getMessageLogger().startMessage(MessageType.Error).add("stopMotor() called on an empty MotorGroupController").endMessage();
            return ;
        }

        motors_.get(0).stopMotor() ;
    }

    public double get() {
        if (motors_.size() == 0) {
            getMessageLogger().startMessage(MessageType.Error).add("get() called on an empty MotorGroupController").endMessage();
            return 0.0 ;
        }

        return motors_.get(0).get() ;
    }

    public boolean getInverted() {
        if (motors_.size() == 0) {
            getMessageLogger().startMessage(MessageType.Error).add("getInverted() called on an empty MotorGroupController").endMessage();
            return false ;
        }

        return motors_.get(0).getInverted() ;
    }

    /// \brief Add a new motor to the group
    /// \param ctrl the motor to add to the group
    /// \param inverted if true, the new motor is inverted with respect to the first motor
    public void addMotor(XeroMotorController ctrl, boolean inverted)  {
        if (motors_.size() > 0 && !motors_.get(0).getType().equals(ctrl.getType()))
        {
            getMessageLogger().startMessage(MessageType.Error).add("invalid motor type in addMotor() for existing MotorGroupController").endMessage();            
            return ;
        }

        motors_.add(ctrl) ;

        if (motors_.size() > 1)
            ctrl.follow(motors_.get(0), inverted) ;
    }

    /// \brief Return the current input voltage to the motor controller
    /// \returns the current input voltage to the motor controller    
    public double getInputVoltage()  {
        if (motors_.size() == 0) {
            getMessageLogger().startMessage(MessageType.Error).add("getInputVoltage() called on an empty MotorGroupController").endMessage();
            return 0.0 ;
        }

        return motors_.get(0).getInputVoltage() ;
    }    

    /// \brief Return the motor voltage applied to the motor
    /// \returns the motor voltage applied to the motor        
    public double getAppliedVoltage() {
        if (motors_.size() == 0) {
            getMessageLogger().startMessage(MessageType.Error).add("getAppliedVoltage() called on an empty MotorGroupController").endMessage();
            return 0.0 ;
        }

        return motors_.get(0).getAppliedVoltage() ;
    }

    /// \brief Returns true if the motor controller supports PID loops on the controller
    /// \returns true if the motor controller supports PID loops on the controller    
    public boolean hasPID()  {
        if (motors_.size() == 0) {
            getMessageLogger().startMessage(MessageType.Error).add("hasPID() called on an empty MotorGroupController").endMessage();
            return false ;
        }

        return motors_.get(0).hasPID() ;
    }

    /// \brief Set the target if running a PID loop on the motor controller
    /// \param target the target for the PID loop on the motor controller       
    public void setTarget(double target)  {
        if (motors_.size() == 0) {
            getMessageLogger().startMessage(MessageType.Error).add("setTarget() called on an empty MotorGroupController").endMessage();
            return ;
        }

        motors_.get(0).setTarget(target);
    }

    /// \brief Set the PID parameters for a PID loop running on the motor controller
    /// \param type the type of pid loop (velocity or position)
    /// \param p the proportional parameter for the PID controller
    /// \param i the integral parameter for the PID controller
    /// \param d the derivative parameter for the PID controller
    /// \param f the feed forward parameter for the PID controller
    /// \param outmax the maximum output parameter for the PID controller     
    public void setPID(PidType type, double p, double i, double d, double f, double outmax)  {
        if (motors_.size() == 0) {
            getMessageLogger().startMessage(MessageType.Error).add("setPID() called on an empty MotorGroupController").endMessage();
            return ;
        }

        motors_.get(0).setPID(type, p, i, d, f, outmax) ;
    }

    /// \brief Stop the PID loop in the motor controller     
    public void stopPID()  {
        if (motors_.size() == 0) {
            getMessageLogger().startMessage(MessageType.Error).add("stopPID() called on an empty MotorGroupController").endMessage();
            return ;
        }

        motors_.get(0).stopPID() ;
    }

    /// \brief Set the factor for converting encoder units to real world units, only applies to the PID loop on the motor controller
    /// \param factor the factor to convert encoder units to real world units       
    public void setPositionConversion(double factor)  {
        if (motors_.size() == 0) {
            getMessageLogger().startMessage(MessageType.Error).add("setPositionConversion() called on an empty MotorGroupController").endMessage();
            return ;
        }

        motors_.get(0).setPositionConversion(factor);
    }

    /// \brief Set the factor for converting encoder units to real world units, only applies to the PID loop on the motor controller
    /// \param factor the factor to convert encoder units to real world units        
    public void setVelocityConversion(double factor)  {
        if (motors_.size() == 0) {
            getMessageLogger().startMessage(MessageType.Error).add("setVelocityConversion() called on an empty MotorGroupController").endMessage();
            return ;
        }

        motors_.get(0).setVelocityConversion(factor);
    }

    /// \brief Set the motor power
    /// \param percent the motor power to assign to the motor      
    public void set(double percent)  {
        if (motors_.size() == 0) {
            getMessageLogger().startMessage(MessageType.Error).add("set() called on an empty MotorGroupController").endMessage();
            return ;
        }

        motors_.get(0).set(percent) ;
    }

    /// \brief Set the motor to invert the direction of motion 
    /// \param inverted if true invert the direction of motion, otherwise do not     
    public void setInverted(boolean inverted)   {
        if (motors_.size() == 0) {
            getMessageLogger().startMessage(MessageType.Error).add("setInverted() called on an empty MotorGroupController").endMessage();
            return ;
        }
            
        motors_.get(0).setInverted(inverted);
    }

    /// \brief Set the neutral mode for the motor
    /// \param mode the neutral mode for the motor      
    public void setNeutralMode(NeutralMode mode)  {
        if (motors_.size() == 0) {
            getMessageLogger().startMessage(MessageType.Error).add("setNeutralMode() called on an empty MotorGroupController").endMessage();
            return ;
        }

        for(XeroMotorController ctrl : motors_)
            ctrl.setNeutralMode(mode);
    }

    /// \brief Set the current motor to follow another motor.  Note the motors must be compatible with each other for following.
    /// \param ctrl the other motor to follow
    /// \param invert if true, follow the other motor but with the power inverted.
    /// \throws MotorRequestFailedException if the motors are not compatible for following.      
    public void follow(XeroMotorController ctrl, boolean invert)  {
        getMessageLogger().startMessage(MessageType.Error).add("follow() not valid on a MotorGroupController").endMessage();
    }

    /// \brief Return a human readable string giving the physical motor controller type
    /// \returns a human readable string giving the physical motor controller type        
    public String getType()  {
        if (motors_.size() == 0)
        {
            getMessageLogger().startMessage(MessageType.Error).add("getType() called on an empty MotorGroupController").endMessage();
            return new String("EmptyMotorGroupController") ;
        }

        return motors_.get(0).getType() ;
    }

    /// \brief Returns true if the motor encoder has an embedded encoder that can return position
    /// \returns true if the motor encoder has an embedded encoder that can return position      
    public boolean hasPosition() {
        if (motors_.size() == 0) {
            getMessageLogger().startMessage(MessageType.Error).add("hasPosition() called on an empty MotorGroupController").endMessage();
            return false ;
        }

        return motors_.get(0).hasPosition() ;
    }

    /// \brief Returns the position of the motor in motor units.  If the setPositionConversion() has been called
    /// then these units will be based on the factor supplied.  Otherwise these units are in encoder ticks.
    /// \returns the position of the motor in motor units        
    public double getPosition() {
        if (motors_.size() == 0) {
            getMessageLogger().startMessage(MessageType.Error).add("getPosition() called on an empty MotorGroupController").endMessage();
            return 0.0 ;
        }

        return motors_.get(0).getPosition() ;  
    }

    /// \brief Reset the encoder values to zero      
    public void resetEncoder() {
        if (motors_.size() == 0) {
            getMessageLogger().startMessage(MessageType.Error).add("resetEncoder() called on an empty MotorGroupController").endMessage();
            return ;
        }

        motors_.get(0).resetEncoder();
    }

    /// \brief Set the current limit for the current supplied to the motor
    /// \param limit the amount of current, in amps,  to the value given   
    public void setCurrentLimit(double limit) {
        if (motors_.size() == 0) {
            getMessageLogger().startMessage(MessageType.Error).add("setCurrentLimit() called on an empty MotorGroupController").endMessage();
            return ;
        }
                    
        for(XeroMotorController ctrl : motors_)
            ctrl.setCurrentLimit(limit);
    }      

    /// \brief Return the firmware version of the motor controller
    /// \returns the firmware version of the motor controller       
    public String getFirmwareVersion() {
        if (motors_.size() == 0)
        {
            getMessageLogger().startMessage(MessageType.Error).add("getFirmwareVersion() called on an empty MotorGroupController").endMessage();
            return new String("EmptyMotorGroupController") ;
        }

        StringBuilder result = new StringBuilder() ;

        for(int i = 0 ; i < motors_.size() ; i++) {
            if (result.length() > 0)
                result.append(",") ;
            
            result.append(motors_.get(i).getFirmwareVersion()) ;
        }

        return result.toString() ;
    }

    /// \brief Set the encoder update frequency.  This configures the rate at which the motor controller
    /// sends back the CAN status packets that contain encoder information form the motor controller to 
    /// the software running on the RoboRio.
    /// \param freq the frequency to update the encoder values        
    public void setEncoderUpdateFrequncy(EncoderUpdateFrequency freq) {
        if (motors_.size() == 0) {
            getMessageLogger().startMessage(MessageType.Error).add("setEncoderUpdateFrequncy() called on an empty MotorGroupController").endMessage();
            return ;
        }

        int which = 0 ;
        for(XeroMotorController ctrl : motors_)
        {
            if (which == 0)
                ctrl.setEncoderUpdateFrequncy(freq);
            else
                ctrl.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Infrequent);
        }
    }
} ;
