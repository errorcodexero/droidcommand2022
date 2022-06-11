package frc.robot.commands;

import java.sql.Driver;

import org.xero1425.misc.ISettingsSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDriveSubsystem;

public class DriveCommand extends CommandBase {

    private class DevicePair {
        private int device_ ;
        private int index_ ;

        public DevicePair(int dev, int ind) {
            device_ = dev ;
            index_ = ind ;
        }

        public int getDevice() {
            return device_ ;
        }

        public int getIndex() {
            return index_ ;
        }
    }

    private DevicePair speed_stick_ ;
    private DevicePair rotate_stick_ ;
    private double deadband_ ;
    private TankDriveSubsystem sub_ ;

    public DriveCommand(TankDriveSubsystem sub) {
        int dev, index ;
        ISettingsSupplier settings = sub.getSettingsSupplier() ;

        sub_ = sub ;
        addRequirements(sub);

        try {
            dev = settings.get("oi:drive:speed:device").getInteger() ;
        }
        catch(Exception ex) {
            dev = 0 ;
        }

        try {
            index = settings.get("oi:drive:speed:index").getInteger();
        }
        catch(Exception ex) {
            index = 1 ;
        }

        speed_stick_ = new DevicePair(dev, index) ;

        try {
            dev = settings.get("oi:drive:rotate:device").getInteger() ;
        }
        catch(Exception ex) {
            dev = 0 ;
        }

        try {
            index = settings.get("oi:drive:rotate:index").getInteger();
        }
        catch(Exception ex) {
            index = 1 ;
        }

        rotate_stick_ = new DevicePair(dev, index) ;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double xSpeed =  DriverStation.getStickAxis(speed_stick_.getDevice(), speed_stick_.getIndex()) ;
        double zRotation =  DriverStation.getStickAxis(rotate_stick_.getDevice(), rotate_stick_.getIndex()) ;

        xSpeed = MathUtil.applyDeadband(xSpeed, deadband_) ;
        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0) ;
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed) ;

        zRotation = MathUtil.applyDeadband(zRotation, deadband_) ;
        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0) ;
        zRotation = Math.copySign(zRotation * zRotation, zRotation) ;       
        
        double leftSpeed, rightSpeed ;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed) ;
        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
              leftSpeed = maxInput;
              rightSpeed = xSpeed - zRotation;
            } else {
              leftSpeed = xSpeed + zRotation;
              rightSpeed = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
              leftSpeed = xSpeed + zRotation;
              rightSpeed = maxInput;
            } else {
              leftSpeed = maxInput;
              rightSpeed = xSpeed - zRotation;
            }
        }

        double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (maxMagnitude > 1.0) {
          leftSpeed /= maxMagnitude;
          rightSpeed /= maxMagnitude;
        }

        sub_.tankDrivePower(leftSpeed, rightSpeed);
    }
}
