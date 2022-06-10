package org.xero1425.base.motors;

import com.ctre.phoenix.ErrorCode;

/// \file
/// This file conatins the implementation of the CTREMotorController class.  This class
/// is derived from the MotorController class and supports the CTRE devices including the TalonFX,
/// the TalonSRX, and the VictorSPX.
///

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotBase;

/// \brief This class is MotorController class that supports the TalonFX motors.   This class is a 
/// wrapper for the TalonFX class that provides the interface that meets the requirements of the 
/// MotorController base class.
public class TalonFXMotorController extends XeroMotorController
{  
    private TalonFX controller_ ;
    private boolean pid_setup_ ;
    private boolean disabled_ ;
    private double power_ ;

    private SimDevice sim_ ;
    private SimDouble sim_power_ ;
    private SimDouble sim_encoder_ ;
    private SimBoolean sim_motor_inverted_ ;
    private SimBoolean sim_neutral_mode_ ;

    /// \brief the name of the device when simulating
    public final static String SimDeviceName = "CTREMotorController" ;

    private final int ControllerTimeout = 100 ;

    /// \brief Create a new TalonFX Motor Controller.
    /// \param name the name of this motor
    /// \param index the CAN address of this motor controller
    public TalonFXMotorController(MessageLogger logger, String name, int index) throws MotorRequestFailedException {
        super(logger, name) ;

        pid_setup_ = false ;
        disabled_ = false ;
        power_ = 0.0 ;

        if (RobotBase.isSimulation()) {
            sim_ = SimDevice.create(SimDeviceName, index) ;

            sim_power_ = sim_.createDouble(XeroMotorController.SimPowerParamName, SimDevice.Direction.kBidir, 0.0) ;
            sim_encoder_ = sim_.createDouble(XeroMotorController.SimEncoderParamName, SimDevice.Direction.kBidir, 0.0) ;
            sim_motor_inverted_ = sim_.createBoolean(XeroMotorController.SimInvertedParamName, SimDevice.Direction.kBidir, false) ;
            sim_neutral_mode_ = sim_.createBoolean(XeroMotorController.SimNeutralParamName, SimDevice.Direction.kBidir, false) ;
            sim_.createBoolean(XeroMotorController.SimEncoderStoresTicksParamName, SimDevice.Direction.kBidir, true) ;

        }
        else {
            ErrorCode code ;

            sim_ = null ;
            sim_power_ = null ;
            sim_encoder_ = null ;

            controller_ = new TalonFX(index) ;

            code = controller_.configFactoryDefault(ControllerTimeout) ;
            if (code != ErrorCode.OK)
                throw new MotorRequestFailedException(this, "CTRE configFactoryDefault() call failed during initialization", code) ;
                
            code = controller_.configVoltageCompSaturation(12.0, ControllerTimeout) ;
            if (code != ErrorCode.OK)
                throw new MotorRequestFailedException(this, "CTRE configVoltageCompSaturation() call failed during initialization", code) ;

            controller_.enableVoltageCompensation(true);

            code = controller_.setSelectedSensorPosition(0, 0, ControllerTimeout) ;
            if (code != ErrorCode.OK)
                throw new MotorRequestFailedException(this, "CTRE setSelectedSensorPosition() call failed during initialization", code) ;

            code = controller_.configNeutralDeadband(0.001, ControllerTimeout);
            if (code != ErrorCode.OK)
                throw new MotorRequestFailedException(this, "CTRE configNeutralDeadband() call failed during initialization", code) ;

            code = controller_.configNominalOutputForward(0, ControllerTimeout) ;
            if (code != ErrorCode.OK)
                throw new MotorRequestFailedException(this, "CTRE configNominalOutputForward() call failed during initialization", code) ;

            code = controller_.configNominalOutputReverse(0, ControllerTimeout) ;
            if (code != ErrorCode.OK)
                throw new MotorRequestFailedException(this, "CTRE configNominalOutputReverse() call failed during initialization", code) ;

            code = controller_.configPeakOutputForward(1, ControllerTimeout) ;
            if (code != ErrorCode.OK)
                throw new MotorRequestFailedException(this, "CTRE configPeakOutputForward() call failed during initialization", code) ;

            code = controller_.configPeakOutputReverse(-1, ControllerTimeout) ;
            if (code != ErrorCode.OK)
                throw new MotorRequestFailedException(this, "CTRE configPeakOutputReverse() call failed during initialization", code) ;               
        }
    }

    public void disable() {
        if (!disabled_) {
            set(0.0) ;
            disabled_ = true ;
        }
    }

    public void stopMotor() {
        set(0.0) ;
    }

    public double get() {
        return power_ ;
    }

    public boolean getInverted() {
        return controller_.getInverted() ;
    }

    /// \brief Return the current input voltage to the motor controller
    /// \returns the current input voltage to the motor controller
    public double getInputVoltage() {
        return controller_.getBusVoltage() ;
    }

    /// \brief Return the motor voltage applied to the motor
    /// \returns the motor voltage applied to the motor    
    public double getAppliedVoltage() {
        return controller_.getMotorOutputVoltage() ;
    }
    
    /// \brief Returns true if the motor controller supports PID loops on the controller
    /// \returns true if the motor controller supports PID loops on the controller
    public boolean hasPID() {
        return true ;
    }

    /// \brief Set the target if running a PID loop on the motor controller
    /// \param target the target for the PID loop on the motor controller    
    public void setTarget(double target) {
        if (pid_setup_ == false) {
            getMessageLogger().startMessage(MessageType.Error).add("calling TalonFXMotorController.setTarget() before calling TalonFXMotorController.setPID()").endMessage();
        }
        else {
            controller_.set(TalonFXControlMode.Velocity, target) ;
        }
    }

    /// \brief Set the PID parameters for a PID loop running on the motor controller
    /// \param type the type of pid loop (velocity or position)
    /// \param p the proportional parameter for the PID controller
    /// \param i the integral parameter for the PID controller
    /// \param d the derivative parameter for the PID controller
    /// \param f the feed forward parameter for the PID controller
    /// \param outmax the maximum output parameter for the PID controller 
    public void setPID(PidType type, double p, double i, double d, double f, double outmax) {
        ErrorCode code ;

        code = controller_.config_kP(0, p, ControllerTimeout) ;
        if (code != ErrorCode.OK) {
            getMessageLogger().startMessage(MessageType.Warning).add("config_kP() failed during TalonFX.setPID() call ").
            add("code", code.toString()).endMessage() ;
        }

        code = controller_.config_kI(0, i, ControllerTimeout) ;
        if (code != ErrorCode.OK) {
            getMessageLogger().startMessage(MessageType.Warning).add("config_kI() failed during TalonFX.setPID() call ").
            add("code", code.toString()).endMessage() ;
        }

        code = controller_.config_kD(0, d, ControllerTimeout) ;
        if (code != ErrorCode.OK) {
            getMessageLogger().startMessage(MessageType.Warning).add("config_kD() failed during TalonFX.setPID() call ").
            add("code", code.toString()).endMessage() ;
        }

        code = controller_.config_kF(0, f, ControllerTimeout) ;
        if (code != ErrorCode.OK) {
            getMessageLogger().startMessage(MessageType.Warning).add("config_kF() failed during TalonFX.setPID() call ").
            add("code", code.toString()).endMessage() ;
        };  

        code = controller_.configClosedLoopPeakOutput(0, outmax, ControllerTimeout) ;
        if (code != ErrorCode.OK) {
            getMessageLogger().startMessage(MessageType.Warning).add("configClosedLoopPeakOutput() failed during TalonFX.setPID() call ").
            add("code", code.toString()).endMessage() ;
        }

        pid_setup_= true ;
    }

    /// \brief Stop the PID loop in the motor controller    
    public void stopPID() {
        controller_.set(ControlMode.PercentOutput, 0.0) ;
    }

    /// \brief Set the factor for converting encoder units to real world units, only applies to the PID loop on the motor controller
    /// \param factor the factor to convert encoder units to real world units    
    public void setPositionConversion(double factor) {
        ErrorCode code = controller_.configSelectedFeedbackCoefficient(factor, 0, ControllerTimeout) ;
        if (code != ErrorCode.OK) {
            getMessageLogger().startMessage(MessageType.Warning).add("configSelectedFeedbackCoefficient() failed during TalonFX.setPositionConversion() call ").
            add("code", code.toString()).endMessage() ;
        }        
    }

    /// \brief Set the factor for converting encoder units to real world units, only applies to the PID loop on the motor controller
    /// \param factor the factor to convert encoder units to real world units     
    public void setVelocityConversion(double factor) {
        ErrorCode code = controller_.configSelectedFeedbackCoefficient(factor, 0, ControllerTimeout) ;
        if (code != ErrorCode.OK) {
            getMessageLogger().startMessage(MessageType.Warning).add("configSelectedFeedbackCoefficient() failed during TalonFX.setVelocityConversion() call ").
            add("code", code.toString()).endMessage() ;
        }    
    }

    /// \brief Set the motor power
    /// \param percent the motor power to assign to the motor    
    public void set(double percent) {
        if (!disabled_) {
            if (sim_ != null) {
                sim_power_.set(percent) ;
            }
            else {
                controller_.set(ControlMode.PercentOutput, percent) ;
            }
        }
    }

    /// \brief Set the motor to invert the direction of motion 
    /// \param inverted if true invert the direction of motion, otherwise do not    
    public void setInverted(boolean inverted) {
        if (sim_ != null) {
            sim_motor_inverted_.set(true) ;
        }
        else {
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
                    controller_.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
                    break ;

                case Brake:
                    controller_.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
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
            if (invert) {
                getMessageLogger().startMessage(MessageType.Error).add("TalonFXMotorController.follow(): cannot follow another controller inverted") ;
                return ;
            }

            try {
                TalonFXMotorController other = (TalonFXMotorController)ctrl ;
                controller_.follow(other.controller_) ;
            }
            catch(ClassCastException ex)
            {
                getMessageLogger().startMessage(MessageType.Error).add("this motor (TalonFXMotorController) cannot follow motor ").addQuoted(ctrl.toString()).endMessage();
            }
        }
    }

    /// \brief Return a human readable string giving the physical motor controller type
    /// \returns a human readable string giving the physical motor controller type    
    public String getType() {
        return "TalonFX" ;
    }

    /// \brief Returns true if the motor encoder has an embedded encoder that can return position
    /// \returns true if the motor encoder has an embedded encoder that can return position    
    public boolean hasPosition() {
        return true ;
    }

    /// \brief Returns the position of the motor in motor units.  If the setPositionConversion() has been called
    /// then these units will be based on the factor supplied.  Otherwise these units are in encoder ticks.
    /// \returns the position of the motor in motor units    
    public double getPosition() {
        double ret = 0 ;

        if (sim_ != null) {
            ret = (int)sim_encoder_.getValue().getDouble() ;
        }
        else {
            TalonFX fx = (TalonFX)controller_ ;
            ret = fx.getSelectedSensorPosition() ;
        }
        
        return ret ;
    }

    /// \brief Reset the encoder values to zero    
    public void resetEncoder() {
        if (sim_ != null) {
            sim_encoder_.set(0.0) ;
        }
        else {
            TalonFX fx = (TalonFX)controller_ ;
            fx.setSelectedSensorPosition(0) ;
        }
    }

    /// \brief Set the current limit for the current supplied to the motor
    /// \param limit the amount of current, in amps,  to the value given    
    public void setCurrentLimit(double limit) {
        if (sim_ == null) {
            TalonFX fx = (TalonFX)controller_ ;
            SupplyCurrentLimitConfiguration cfg = new SupplyCurrentLimitConfiguration(true, limit, limit, 1) ;
            fx.configSupplyCurrentLimit(cfg) ;
        }
    }     
    
    /// \brief Return the firmware version of the motor controller
    /// \returns the firmware version of the motor controller    
    public String getFirmwareVersion() {
        int v = controller_.getFirmwareVersion() ;

        return String.valueOf((v >> 8) & 0xff) + "." + String.valueOf(v & 0xff) ;
    }

    /// \brief Set the encoder update frequency.  This configures the rate at which the motor controller
    /// sends back the CAN status packets that contain encoder information form the motor controller to 
    /// the software running on the RoboRio.
    /// \param freq the frequency to update the encoder values    
    public void setEncoderUpdateFrequncy(EncoderUpdateFrequency freq) {
        if (controller_ != null)
        {
            if (freq == EncoderUpdateFrequency.Infrequent) {
                controller_.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 500) ;
                controller_.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 500) ;
            }
            else if (freq == EncoderUpdateFrequency.Default) {
                controller_.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10) ;
                controller_.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20) ;            
            }
            else if (freq == EncoderUpdateFrequency.Frequent) {
                controller_.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10) ;
                controller_.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10) ;             
            }
        }        
    }     
} ;
