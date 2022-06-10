package org.xero1425.base;

import org.xero1425.base.motors.MotorFactory;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.SettingsValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class XeroSubsystem extends SubsystemBase {
    private XeroRobot robot_ ;
    private String name_ ;
    private int logger_id_ ;
    private double last_time_ ;
    private double delta_time_ ;

    public XeroSubsystem(XeroRobot robot, String name) {
        robot_ = robot ;

        logger_id_ = robot.getMessageLogger().registerSubsystem(name) ;
        last_time_ = 0.0 ;
        delta_time_ = 0.0 ;
    }

    public int getLoggerID() {
        return logger_id_ ;
    }

    public MessageLogger getMessageLogger() {
        return robot_.getMessageLogger() ;
    }

    public MotorFactory getMotorFactory() {
        return robot_.getMotorFactory() ;
    }
    
    public SettingsValue getSetting(String name) throws MissingParameterException {
        String settings = "subsystems:" + name_ + ":" + name ;
        return robot_.getSettingsSupplier().get(settings) ;
    }

    public double getDeltaTime() {
        return delta_time_ ;
    }

    @Override
    public void periodic() {
        double now = robot_.getTime() ;
        delta_time_ = now - last_time_ ;
        last_time_ = now ;
    }
}
