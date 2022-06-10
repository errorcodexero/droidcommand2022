package org.xero1425.base.motors;

/// \file
/// This file conatins the implementation of the CTREMotorController class.  This class
/// is derived from the MotorController class and supports the CTRE devices including the TalonFX,
/// the TalonSRX, and the VictorSPX.
///

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.ErrorCode;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotBase;

/// \brief This class is MotorController class that supports the TalonSRX, and the VictorSPX motors.
public class CTREMotorController extends XeroMotorController
{  
    private BaseMotorController controller_ ;
    private boolean inverted_ ;
    private MotorType type_ ;
    private double power_ ;
    private boolean disabled_ ;

    private SimDevice sim_ ;
    private SimDouble sim_power_ ;
    private SimBoolean sim_motor_inverted_ ;
    private SimBoolean sim_neutral_mode_ ;

    /// \brief The name of the device when simulating
    public final static String SimDeviceName = "CTREMotorController" ;

    private final int ControllerTimeout = 100 ;

    /// \brief The type of the physical motor controller
    public enum MotorType
    {
        TalonSRX,           ///< A Talon SRX motor controller
        VictorSPX,          ///< A Victor SPX motor controller
    } ;
    
    /// \brief Create a new Talon SRX or Victor SPX Motor Controller.
    /// \param name the name of this motor
    /// \param index the CAN address of this motor controller
    /// \param type the type of the motor controller
    public CTREMotorController(MessageLogger logger, String name, int index, MotorType type) throws MotorRequestFailedException {
        super(logger, name) ;

        inverted_ = false ;
        type_ = type ;

        if (RobotBase.isSimulation()) {
            sim_ = SimDevice.create(SimDeviceName, index) ;

            sim_power_ = sim_.createDouble(XeroMotorController.SimPowerParamName, SimDevice.Direction.kBidir, 0.0) ;
            sim_motor_inverted_ = sim_.createBoolean(XeroMotorController.SimInvertedParamName, SimDevice.Direction.kBidir, false) ;
            sim_neutral_mode_ = sim_.createBoolean(XeroMotorController.SimNeutralParamName, SimDevice.Direction.kBidir, false) ;
            sim_.createBoolean(XeroMotorController.SimEncoderStoresTicksParamName, SimDevice.Direction.kBidir, true) ;

        }
        else {
            ErrorCode code ;

            sim_ = null ;
            sim_power_ = null ;
            disabled_ = false ;

            switch(type_)
            {
                case TalonSRX:
                    controller_ = new TalonSRX(index) ;
                    break ;

                case VictorSPX:
                    controller_ = new VictorSPX(index) ;
                    break ;
            }

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
        return inverted_ ;
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
        getMessageLogger().startMessage(MessageType.Error).add("setTarget() not implemented for CTREMotorController").endMessage();
    }

    /// \brief Set the PID parameters for a PID loop running on the motor controller
    /// \param type the type of pid loop (velocity or position)
    /// \param p the proportional parameter for the PID controller
    /// \param i the integral parameter for the PID controller
    /// \param d the derivative parameter for the PID controller
    /// \param f the feed forward parameter for the PID controller
    /// \param outmax the maximum output parameter for the PID controller     
    public void setPID(PidType type, double p, double i, double d, double f, double outmax) {
        getMessageLogger().startMessage(MessageType.Error).add("setPID() not implemented for CTREMotorController").endMessage();
    }

    /// \brief Stop the PID loop in the motor controller     
    public void stopPID() {
        getMessageLogger().startMessage(MessageType.Error).add("stopPID() not implemented for CTREMotorController").endMessage();
    }

    /// \brief Set the factor for converting encoder units to real world units, only applies to the PID loop on the motor controller
    /// \param factor the factor to convert encoder units to real world units      
    public void setPositionConversion(double factor) {
        getMessageLogger().startMessage(MessageType.Error).add("setPositionConversion() not implemented for CTREMotorController").endMessage();        
    }

    /// \brief Set the factor for converting encoder units to real world units, only applies to the PID loop on the motor controller
    /// \param factor the factor to convert encoder units to real world units   
    public void setVelocityConversion(double factor) {
        getMessageLogger().startMessage(MessageType.Error).add("setVelocityConversion() not implemented for CTREMotorController").endMessage();        
    }

    /// \brief Set the motor power
    /// \param percent the motor power to assign to the motor  
    public void set(double percent) {

        if (!disabled_) {
            power_ = percent ;
            
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
        inverted_ = inverted ;
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
            try {
                CTREMotorController other = (CTREMotorController)ctrl ;
                controller_.follow(other.controller_) ;
            }
            catch(ClassCastException ex)
            {
                getMessageLogger().startMessage(MessageType.Error).add("this motor (CTREMotorController) cannot follow motor ").addQuoted(ctrl.toString()).endMessage();
            }
        }
    }

    /// \brief Return a human readable string giving the physical motor controller type
    /// \returns a human readable string giving the physical motor controller type 
    public String getType() {
        String ret = null ;

        switch(type_)
        {
            case TalonSRX:
                ret = "TalonSRX" ;
                break ;

            case VictorSPX:
                ret = "VictorSPX" ;
                break ;
        }

        return ret ;
    }

    /// \brief Returns true if the motor encoder has an embedded encoder that can return position
    /// \returns true if the motor encoder has an embedded encoder that can return position 
    public boolean hasPosition() {
        return false ;
    }

    /// \brief Set the current limit for the current supplied to the motor
    /// \param limit the amount of current, in amps,  to the value given    
    public void setCurrentLimit(double limit) {
        if (sim_ == null) {
            if (controller_ instanceof TalonSRX)
            {
                TalonSRX srx = (TalonSRX)controller_ ;
                SupplyCurrentLimitConfiguration cfg = new SupplyCurrentLimitConfiguration();
                cfg.currentLimit = limit ;
                cfg.enable = true ;
                srx.configSupplyCurrentLimit(cfg) ;
            }
            else
            {
                getMessageLogger().startMessage(MessageType.Error).add("setCurrentLimit() not implemented for CTREMotorController(VictorSPX)").endMessage();
            }
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
        getMessageLogger().startMessage(MessageType.Error).add("setEncoderUpdateFrequncy() not implemented for CTREMotorController").endMessage();
    }
} ;

