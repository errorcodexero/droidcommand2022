package org.xero1425.base.motors;

import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

/// \file

/// \brief This class is MotorController class that supports the Romi motor controller.   This class is a 
/// wrapper for the Spark class that provides the interface that meets the requirements of the 
/// MotorController base class.  The Romi robot is always run as a simulation, so under the hood this Spark
/// motor controller, when simulated, is monitored and its output power sent to the Romi robot over the 
/// WI-FI link.
public class RomiMotorController extends XeroMotorController {

    private Spark motor_ ;
    private boolean inverted_ ;

    /// \brief Create a new Romi Motor Controller.
    /// \param name the name of this motor
    /// \param index the CAN address of this motor controller    
    public RomiMotorController(MessageLogger logger, String name, int index) {
        super(logger, name) ;

        motor_ = new Spark(index) ;
    }

    public void disable() {
        motor_.disable(); 
    }

    public double get() {
        return motor_.get() ;
    }

    public boolean getInverted() {
        return inverted_ ;
    }

    public void stopMotor() {
        motor_.stopMotor(); ;
    }

    /// \brief Set the motor power
    /// \param percent the motor power to assign to the motor        
    public void set(double percent) {
        motor_.set(percent) ;
    }

    /// \brief Set the motor to invert the direction of motion 
    /// \param inverted if true invert the direction of motion, otherwise do not       
    public void setInverted(boolean inverted)  {
        inverted_ = inverted ;
        motor_.setInverted(inverted);
    }

    /// \brief Reapplies the inverted status of the motor.  When setInverted() is called, the inverted state of the motor
    /// is stored and this method reapplies that stored state to the motor controller.  This was put into place because some
    /// motors setup to follow other motors lost their inverted state when the robot was disabled and re-enabled.      
    public void reapplyInverted() {
        motor_.setInverted(inverted_);
    }

    /// \brief Set the neutral mode for the motor
    /// \param mode the neutral mode for the motor 
    public void setNeutralMode(NeutralMode mode) {
        getMessageLogger().startMessage(MessageType.Error).add("setNeutralMode() not implemented for CTREMotorController").endMessage();             
    }

    /// \brief Return a human readable string giving the physical motor controller type
    /// \returns a human readable string giving the physical motor controller type        
    public String getType()  {
        return "Romi" ;
    }

    /// \brief Return the current input voltage to the motor controller
    /// \returns the current input voltage to the motor controller     
    public double getInputVoltage() {
        getMessageLogger().startMessage(MessageType.Error).add("getInputVoltage() not implemented for CTREMotorController").endMessage();  
        return 0.0 ;
    }

    /// \brief Return the motor voltage applied to the motor
    /// \returns the motor voltage applied to the motor       
    public double getAppliedVoltage() {
        getMessageLogger().startMessage(MessageType.Error).add("getAppliedVoltage() not implemented for CTREMotorController").endMessage();  
        return 0.0 ;
    }

    /// \brief Returns true if the motor controller supports PID loops on the controller.  Note
    /// always returns false for the Romi motor controller
    /// \returns true if the motor controller supports PID loops on the controller    
    public boolean hasPID() {
        return false ;
    }

    /// \brief Set the target if running a PID loop on the motor controller
    /// \param target the target for the PID loop on the motor controller
    /// \throws BadMotorRequestException always since PID loops are not supported on the Romi controller
    public void setTarget(double target) {
        getMessageLogger().startMessage(MessageType.Error).add("setTarget() not implemented for CTREMotorController").endMessage();  
    }

    /// \brief Stop the PID loop in the motor controller     
    /// \throws BadMotorRequestException always since PID loops are not supported on the Romi controller    
    public void stopPID() {
        getMessageLogger().startMessage(MessageType.Error).add("stopPID() not implemented for CTREMotorController").endMessage();  
    }

    /// \brief Set the PID parameters for a PID loop running on the motor controller.  Note, this has not been fully
    /// implemented or tested for the SparkMax motor controller.  Note, always throws an exception.
    /// \param type the type of pid loop (velocity or position)
    /// \param p the proportional parameter for the PID controller
    /// \param i the integral parameter for the PID controller
    /// \param d the derivative parameter for the PID controller
    /// \param f the feed forward parameter for the PID controller
    /// \param outmax the maximum output parameter for the PID controller       
    /// \throws BadMotorRequestException always since PID loops are not supported on the Romi controller    
    public void setPID(PidType type, double p, double i, double d, double f, double outmax) {
        getMessageLogger().startMessage(MessageType.Error).add("setPID() not implemented for CTREMotorController").endMessage();  
    }

    /// \brief Set the current motor to follow another motor.  Note the motors must be compatible with each other for following.
    /// \param ctrl the other motor to follow
    /// \param invert if true, follow the other motor but with the power inverted.
    /// \throws BadMotorRequestException since the Romi motor controller does not support following
    public void follow(XeroMotorController ctrl, boolean invert) {
        getMessageLogger().startMessage(MessageType.Error).add("follow() not implemented for CTREMotorController").endMessage();  
    }

    /// \brief Set the factor for converting encoder units to real world units, only applies to the PID loop on the motor controller
    /// \param factor the factor to convert encoder units to real world units        
    /// \throws BadMotorRequestException always since PID loops are not supported on the Romi controller       
    public void setPositionConversion(double factor) {
        getMessageLogger().startMessage(MessageType.Error).add("setPositionConversion() not implemented for CTREMotorController").endMessage();  
    }

    /// \brief Set the factor for converting encoder units to real world units, only applies to the PID loop on the motor controller
    /// \param factor the factor to convert encoder units to real world units       
    /// \throws BadMotorRequestException always since PID loops are not supported on the Romi controller     
    public void setVelocityConversion(double factor) {
        getMessageLogger().startMessage(MessageType.Error).add("setVelocityConversion() not implemented for CTREMotorController").endMessage();  
    }

    /// \brief Return the firmware version of the motor controller
    /// \returns the firmware version of the motor controller     
    public String getFirmwareVersion() {
        return "?.?" ;
    }
    
    /// \brief Set the encoder update frequency.  This configures the rate at which the motor controller
    /// sends back the CAN status packets that contain encoder information form the motor controller to 
    /// the software running on the RoboRio.
    /// \param freq the frequency to update the encoder values     
    public void setEncoderUpdateFrequncy(EncoderUpdateFrequency freq) {
        getMessageLogger().startMessage(MessageType.Error).add("setEncoderUpdateFrequncy() not implemented for CTREMotorController").endMessage();  
    }
}
