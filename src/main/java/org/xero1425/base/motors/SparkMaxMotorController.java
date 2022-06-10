package org.xero1425.base.motors;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotBase;

/// \file

/// \brief This class is MotorController class that supports the SparkMax motor controller.   This class is a 
/// wrapper for the CANSparkMax class that provides the interface that meets the requirements of the 
/// MotorController base class.  This class supports both brushless and brushed motors.
public class SparkMaxMotorController extends XeroMotorController
{
    private CANSparkMax controller_ ;
    private RelativeEncoder encoder_ ;
    private boolean brushless_ ;
    private SparkMaxPIDController pid_ ;
    private PidType ptype_ ;
    private double target_ ;

    private SimDevice sim_ ;
    private SimDouble sim_power_ ;
    private SimDouble sim_encoder_ ;
    private SimBoolean sim_motor_inverted_ ;
    private SimBoolean sim_neutral_mode_ ;

    /// \brief The device name in simulation for a brushed motor
    public final static String SimDeviceNameBrushed = "SparkMaxBrushed" ;

    /// \brief The device name in simulation for a brushless motor    
    public final static String SimDeviceNameBrushless = "SparkMaxBrushless" ;

    /// \brief A constant that gives the number of ticks per revolution for brushless motors
    public final static int TicksPerRevolution = 42 ;

    /// \brief Create a new SparkMax Motor Controller.
    /// \param name the name of this motor
    /// \param index the CAN address of this motor controller    
    /// \param brushless if true, the motor is a brushless motgor
    public SparkMaxMotorController(MessageLogger logger, String name, int index, boolean brushless) throws MotorRequestFailedException {
        super(logger, name) ;

        brushless_ = brushless ;
        pid_ = null ;
        target_ = 0 ;

        if (RobotBase.isSimulation()) {
            if (brushless)
                sim_ = SimDevice.create(SimDeviceNameBrushless, index) ;
            else
                sim_ = SimDevice.create(SimDeviceNameBrushed, index) ;

            sim_power_ = sim_.createDouble(XeroMotorController.SimPowerParamName, SimDevice.Direction.kBidir, 0.0) ;
            sim_encoder_ = sim_.createDouble(XeroMotorController.SimEncoderParamName, SimDevice.Direction.kBidir, 0.0) ;
            sim_motor_inverted_ = sim_.createBoolean(XeroMotorController.SimInvertedParamName, SimDevice.Direction.kBidir, false) ;
            sim_neutral_mode_ = sim_.createBoolean(XeroMotorController.SimNeutralParamName, SimDevice.Direction.kBidir, false) ;  
            sim_.createBoolean(XeroMotorController.SimEncoderStoresTicksParamName, SimDevice.Direction.kBidir, false) ;            
        }
        else {
            REVLibError code ;

            if (brushless)
            {
                controller_ = new CANSparkMax(index, CANSparkMax.MotorType.kBrushless) ;
            }
            else
            {
                controller_ = new CANSparkMax(index, CANSparkMax.MotorType.kBrushed) ;
            }

            code = controller_.restoreFactoryDefaults() ;
            if (code != REVLibError.kOk)
                throw new MotorRequestFailedException(this, "restoreFactoryDefaults() failed during initialization", code) ;

            code = controller_.enableVoltageCompensation(12.0) ;
            if (code != REVLibError.kOk)
                throw new MotorRequestFailedException(this, "enableVoltageCompensation() failed during initialization", code) ;

            encoder_ = controller_.getEncoder() ;
        }
    }

    public void disable() {
        controller_.disable();
    }

    public void stopMotor() {
        controller_.stopMotor();
    }

    public double get() {
        return controller_.get() ;
    }

    public boolean getInverted() {
        boolean ret = false ;

        if (sim_ != null) {
            ret = sim_motor_inverted_.get() ;
        } else {
            ret = controller_.getInverted() ;
        }

        return ret ;
    }

    /// \brief Return the current input voltage to the motor controller
    /// \returns the current input voltage to the motor controller    
    public double getInputVoltage() {
        if (RobotBase.isSimulation())
            return 12.0 ;

        return controller_.getBusVoltage() ;
    }

    /// \brief Return the motor voltage applied to the motor
    /// \returns the motor voltage applied to the motor      
    public double getAppliedVoltage() {
        if (RobotBase.isSimulation())
            return 12.0 ;

        return controller_.getAppliedOutput() ;
    }

    /// \brief Returns true if the motor controller supports PID loops on the controller
    /// \returns true if the motor controller supports PID loops on the controller
    public boolean hasPID() {
        if (RobotBase.isSimulation())
            return false ;
            
        return true ;
    }

    /// \brief Set the target if running a PID loop on the motor controller
    /// \param target the target for the PID loop on the motor controller     
    public void setTarget(double target)  {
        REVLibError code = REVLibError.kOk ;

        target_ = target ;

        if (pid_ != null) {

            if (ptype_ == PidType.Position)
                code = pid_.setReference(target, CANSparkMax.ControlType.kPosition) ;
            else if (ptype_ == PidType.Velocity)
                code = pid_.setReference(target, CANSparkMax.ControlType.kVelocity) ;
            
            if (code != REVLibError.kOk) {
                getMessageLogger().startMessage(MessageType.Warning).add("setReference() failed during SparkMaxMotorController.setTarget() call ").
                        add("code", code.toString()).endMessage() ;
            }
        }
    }

    /// \brief Set the PID parameters for a PID loop running on the motor controller.  Note, this has not been fully
    /// implemented or tested for the SparkMax motor controller.
    /// \param type the type of pid loop (velocity or position)
    /// \param p the proportional parameter for the PID controller
    /// \param i the integral parameter for the PID controller
    /// \param d the derivative parameter for the PID controller
    /// \param f the feed forward parameter for the PID controller
    /// \param outmax the maximum output parameter for the PID controller     
    public void setPID(PidType type, double p, double i, double d, double f, double outmax) {
        REVLibError code = REVLibError.kOk ;

        if (pid_ == null)
            pid_ = controller_.getPIDController() ;

        code = pid_.setP(p) ;
        if (code != REVLibError.kOk) {
            getMessageLogger().startMessage(MessageType.Warning).add("setP() failed during SparkMaxMotorController.setPID() call ").
            add("code", code.toString()).endMessage() ;
        }

        code = pid_.setI(i) ;
        if (code != REVLibError.kOk) {
            getMessageLogger().startMessage(MessageType.Warning).add("setI() failed during SparkMaxMotorController.setPID() call ").
            add("code", code.toString()).endMessage() ;
        }   

        code = pid_.setD(d) ;
        if (code != REVLibError.kOk) {
            getMessageLogger().startMessage(MessageType.Warning).add("setD() failed during SparkMaxMotorController.setPID() call ").
            add("code", code.toString()).endMessage() ;
        }

        code = pid_.setFF(f) ;
        if (code != REVLibError.kOk) {
            getMessageLogger().startMessage(MessageType.Warning).add("setFF() failed during SparkMaxMotorController.setPID() call ").
            add("code", code.toString()).endMessage() ;
        }

        code = pid_.setIZone(0.0) ;
        if (code != REVLibError.kOk) {
            getMessageLogger().startMessage(MessageType.Warning).add("setIZone() failed during SparkMaxMotorController.setPID() call ").
            add("code", code.toString()).endMessage() ;
        }

        code = pid_.setOutputRange(-outmax, outmax) ;
        if (code != REVLibError.kOk) {
            getMessageLogger().startMessage(MessageType.Warning).add("setOutputRange() failed during SparkMaxMotorController.setPID() call ").
            add("code", code.toString()).endMessage() ;
        }

        ptype_ = type ;
        setTarget(target_) ;
    }

    /// \brief Stop the PID loop in the motor controller      
    public void stopPID() {
    }

    /// \brief Set the factor for converting encoder units to real world units, only applies to the PID loop on the motor controller
    /// \param factor the factor to convert encoder units to real world units    
    public void setPositionConversion(double factor) {
        if (!RobotBase.isSimulation()) {
            encoder_.setPositionConversionFactor(factor) ;
        }
    }

    /// \brief Set the factor for converting encoder units to real world units, only applies to the PID loop on the motor controller
    /// \param factor the factor to convert encoder units to real world units     
    public void setVelocityConversion(double factor) {
        if (!RobotBase.isSimulation()) {
            encoder_.setVelocityConversionFactor(factor) ;
        }
    }

    /// \brief Set the motor power
    /// \param percent the motor power to assign to the motor     
    public void set(double percent) {
        if (sim_ != null) {
            sim_power_.set(percent) ;
        } else {
            controller_.set(percent) ;
        }
    }

    /// \brief Set the motor to invert the direction of motion 
    /// \param inverted if true invert the direction of motion, otherwise do not      
    public void setInverted(boolean inverted) {
        if (sim_ != null) {
            sim_motor_inverted_.set(inverted) ;
        } else {
            controller_.setInverted(inverted);
        }
    }

    /// \brief Set the neutral mode for the motor
    /// \param mode the neutral mode for the motor        
    public void setNeutralMode(NeutralMode mode) {
        if (sim_ != null) {
            switch(mode)
            {
                case Coast:
                    sim_neutral_mode_.set(false) ;
                    break ;

                case Brake:
                    sim_neutral_mode_.set(true) ;
                    break ;
            }
        }
        else {
            switch(mode)
            {
                case Coast:
                    controller_.setIdleMode(IdleMode.kCoast) ;
                    break ;

                case Brake:
                    controller_.setIdleMode(IdleMode.kBrake) ;
                break ;
            }
        }
    }

    /// \brief Set the current motor to follow another motor.  Note the motors must be compatible with each other for following.
    /// \param ctrl the other motor to follow
    /// \param invert if true, follow the other motor but with the power inverted.
    /// \throws MotorRequestFailedException if the motors are not compatible for following.      
    public void follow(XeroMotorController ctrl, boolean invert) {
        if (sim_ == null) {
            try {
                SparkMaxMotorController other = (SparkMaxMotorController)ctrl ;
                controller_.follow(other.controller_, invert) ;
            }
            catch(ClassCastException ex)
            {
                getMessageLogger().startMessage(MessageType.Error).add("this motor (SparkMaxMotorController) cannot follow motor ").addQuoted(ctrl.toString()).endMessage();
            }
        }
    }

    /// \brief Return a human readable string giving the physical motor controller type
    /// \returns a human readable string giving the physical motor controller type     
    public String getType() {
        String ret = null ;

        if (brushless_)
        {
            ret = "SparkMax:brushless" ;
        }
        else
        {
            ret = "SparkMax:brushed" ;
        }

        return ret ;
    }

    /// \brief Returns true if the motor encoder has an embedded encoder that can return position
    /// \returns true if the motor encoder has an embedded encoder that can return position      
    public boolean hasPosition() {
        return brushless_ ;
    }

    /// \brief Returns the position of the motor in motor units.  If the setPositionConversion() has been called
    /// then these units will be based on the factor supplied.  Otherwise these units are in encoder ticks.
    /// \returns the position of the motor in motor units        
    public double getPosition() {
        double ret = 0 ;

        if (!brushless_) {
            getMessageLogger().startMessage(MessageType.Warning).add("SparkMaxMotorController.getPosition() not valid with brushless motor").endMessage();
        }
        else if (sim_ != null) {
            ret = sim_encoder_.get() * (double)TicksPerRevolution ;
        } else {
            ret = encoder_.getPosition() * TicksPerRevolution ;
        }

        return ret ;
    }

    /// \brief Reset the encoder values to zero
    public void resetEncoder() {
        if (!brushless_) {
            getMessageLogger().startMessage(MessageType.Warning).add("SparkMaxMotorController.getPosition() not valid with brushless motor").endMessage();
        }
        else if (sim_ != null) {
            sim_encoder_.set(0.0) ;
        }
        else {
            encoder_.setPosition(0.0) ;
        }
    }

    /// \brief Set the current limit for the current supplied to the motor
    /// \param limit the amount of current, in amps,  to the value given        
    public void setCurrentLimit(double limit) {
        if (sim_ == null) {
            controller_.setSmartCurrentLimit((int)limit) ;
        }
    }      

    /// \brief Return the firmware version of the motor controller
    /// \returns the firmware version of the motor controller        
    public String getFirmwareVersion() {
        int v = controller_.getFirmwareVersion() ;

        return String.valueOf((v >> 24) & 0xff) + "." + String.valueOf((v >> 16) & 0xff) ;
    }

    /// \brief Set the encoder update frequency.  This configures the rate at which the motor controller
    /// sends back the CAN status packets that contain encoder information form the motor controller to 
    /// the software running on the RoboRio.
    /// \param freq the frequency to update the encoder values     
    public void setEncoderUpdateFrequncy(EncoderUpdateFrequency freq) {
        if (freq == EncoderUpdateFrequency.Infrequent) {
            controller_.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100) ;
            controller_.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 100) ;
            controller_.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 100) ;        
        }
        else if (freq == EncoderUpdateFrequency.Default) {
            controller_.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10) ;
            controller_.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20) ;
            controller_.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 50) ;  
        }
        else if (freq == EncoderUpdateFrequency.Frequent) {
            controller_.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10) ;
            controller_.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20) ;
            controller_.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 10) ; 
        }        
    }    
} ;
