package org.xero1425.base.motors ;

import org.xero1425.misc.MessageLogger;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/// \file

/// \brief This class is an abstract base class that defines a contract that all supported motors
/// must meet.  There are specific derived classes for Talon SRX, Victor SPX, Talon Fx, and SparkMax
/// motor controller.  There are also derived classes where a group of mechanically connected motors
/// are represented by a single MotorController derived object (MotorGroupController).
public abstract class XeroMotorController implements MotorController
{
    // The name of the motor
    private String name_ ;

    // The message logger so that motors can print messages
    MessageLogger logger_ ;

    /// \brief Property name for property used for motor power in a simulation
    public final static String SimPowerParamName = "Power" ;

    /// \brief Property name for property used for the encoder value in a simulation    
    public final static String SimEncoderParamName = "Encoder" ;

    /// \brief Property name for property used for the stores ticks in a simulation (i.e. motor contains encoder)
    public final static String SimEncoderStoresTicksParamName = "StoresTicks" ;

    /// \brief Property name for property used for to indicate if the power should be inverted in a simulation
    public final static String SimInvertedParamName = "Inverted" ;

     /// \brief Property name for property used for to indicate the neutral mode in a simulation  
    public final static String SimNeutralParamName = "Neutral" ;

    /// \brief The NeutralMode for the motor
    public enum NeutralMode { 
        Coast,              ///< Coast mode
        Brake               ///< Brake mode
    } ;

    /// \brief This enumeration defines how frequencly encoder should be updated
    public enum EncoderUpdateFrequency {
        Frequent,           ///< Encoders should be updated as frequently as possible (once per robot loop)
        Default,            ///< Encoders should be updated at a nominal rate (every few robot loops)
        Infrequent          ///< Encoders are not sample frequently, once every second or two
    } ;

    /// \brief Create a new motor controller
    /// \param name the name of the motor controller
    XeroMotorController(MessageLogger logger, String name) {
        logger_ = logger ;
        name_ = name ;
    }

    public MessageLogger getMessageLogger() {
        return logger_ ;
    }

    /// \brief Returns the name of the motor controller
    /// \returns the name of the motor controller
    public String getName() {
        return name_  ;
    }

    /// \brief PidType the type of PID control to run on the motor controller
    public enum PidType {
        Position,                   ///< Position PID control
        Velocity,                   ///< Velocity PID control
    }

    /// \brief Disable the current motor
    public abstract void disable() ;

    /// \brief Set the motor power
    /// \param percent the motor power to assign to the motor
    public abstract void set(double percent) ;
    
    /// \brief Set the motor to invert the direction of motion 
    /// \param inverted if true invert the direction of motion, otherwise do not
    public abstract void setInverted(boolean inverted) ;

    /// \brief Set the neutral mode for the motor
    /// \param mode the neutral mode for the motor
    public abstract void setNeutralMode(NeutralMode mode) ;

    /// \brief Set the current motor to follow another motor.  Note the motors must be compatible with each other for following.
    /// \param ctrl the other motor to follow
    /// \param invert if true, follow the other motor but with the power inverted.
    /// \throws MotorRequestFailedException if the motors are not compatible for following.
    public abstract void follow(XeroMotorController ctrl, boolean invert) ;

    /// \brief Return a human readable string giving the physical motor controller type
    /// \returns a human readable string giving the physical motor controller type
    public abstract String getType() ;

    /// \brief Return the current input voltage to the motor controller
    /// \returns the current input voltage to the motor controller
    public abstract double getInputVoltage() ;
    
    /// \brief Returns true if the motor controller supports PID loops on the controller
    /// \returns true if the motor controller supports PID loops on the controller
    public abstract boolean hasPID() ;

    /// \brief Set the target if running a PID loop on the motor controller
    /// \param target the target for the PID loop on the motor controller
    public abstract void setTarget(double target) ;

    /// \brief Set the PID parameters for a PID loop running on the motor controller
    /// \param type the type of pid loop (velocity or position)
    /// \param p the proportional parameter for the PID controller
    /// \param i the integral parameter for the PID controller
    /// \param d the derivative parameter for the PID controller
    /// \param f the feed forward parameter for the PID controller
    /// \param outmax the maximum output parameter for the PID controller 
    public abstract void setPID(PidType type, double p, double i, double d, double f, double outmax) ;

    /// \brief Stop the PID loop in the motor controller
    public abstract void stopPID() ;

    /// \brief Set the factor for converting encoder units to real world units, only applies to the PID loop on the motor controller
    /// \param factor the factor to convert encoder units to real world units
    public abstract void setPositionConversion(double factor) ;

    /// \brief Set the factor for converting encoder units to real world units, only applies to the PID loop on the motor controller
    /// \param factor the factor to convert encoder units to real world units    
    public abstract void setVelocityConversion(double factor) ;

    /// \brief Return the firmware version of the motor controller
    /// \returns the firmware version of the motor controller
    public abstract String getFirmwareVersion() ;

    /// \brief Return the motor voltage applied to the motor
    /// \returns the motor voltage applied to the motor
    public abstract double getAppliedVoltage() ;

    /// \brief Set the encoder update frequency.  This configures the rate at which the motor controller
    /// sends back the CAN status packets that contain encoder information form the motor controller to 
    /// the software running on the RoboRio.
    /// \param freq the frequency to update the encoder values
    public abstract void setEncoderUpdateFrequncy(EncoderUpdateFrequency freq) ;

    /// \brief Returns true if the motor encoder has an embedded encoder that can return position
    /// \returns true if the motor encoder has an embedded encoder that can return position
    public boolean hasPosition() {
        return false ;
    }

    /// \brief Returns the position of the motor in motor units.  If the setPositionConversion() has been called
    /// then these units will be based on the factor supplied.  Otherwise these units are in encoder ticks.
    /// \returns the position of the motor in motor units
    public double getPosition()  {
        return 0.0 ;
    }

    /// \brief Reset the encoder values to zero
    public void resetEncoder()  {
    }

    /// \brief Set the current limit for the current supplied to the motor
    /// \param limit the amount of current, in amps,  to the value given
    public void setCurrentLimit(double limit)  {
    }
}
