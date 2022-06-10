package frc.robot.subsystems;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.XeroSubsystem;
import org.xero1425.base.gyro.NavxGyro;
import org.xero1425.base.gyro.XeroGyro;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.XeroMotorController;
import org.xero1425.base.motors.XeroMotorController.EncoderUpdateFrequency;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class TankDriveSubsystem extends XeroSubsystem {
    private double width_ ;
    private XeroGyro gyro_ ;
    private DifferentialDriveOdometry odometry_ ;
    private DifferentialDrive drive_ ;

    private double ticks_per_meter_ ;

    private double ticks_left_ ;
    private double ticks_right_ ;
    private double dist_left_ ;
    private double dist_right_ ;
    private double last_dist_left_ ;
    private double last_dist_right_ ;
    private double speed_left_ ;
    private double speed_right_ ;

    private XeroMotorController left_motors_ ;
    private XeroMotorController right_motors_ ;
    private Encoder left_encoder_ ;
    private Encoder right_encoder_ ;

    public TankDriveSubsystem(XeroRobot robot, String name) throws Exception {
        super(robot, name) ;

        if (!createGyro())
            throw new Exception("cannot create gyro") ;

        try {
            attachHardware() ;
        }
        catch(Exception ex) {
            getMessageLogger().startMessage(MessageType.Error).add("cannot create subsystem ").addQuoted(name).add(" of type ").addQuoted("TankDriveSubsystem") ;
            throw ex ;
        }

        width_ = getSetting("physical:width").getDouble() ;
        ticks_per_meter_ = getSetting("physical:ticks-per-meter").getDouble() ;
        odometry_ = new DifferentialDriveOdometry(getHeading()) ;

        drive_ = new DifferentialDrive(left_motors_, right_motors_) ;

        last_dist_left_ = 0.0 ;
        last_dist_right_ = 0.0 ;
        resetEncoders();
    }

    @Override
    public void periodic() {
        super.periodic();

        if (left_motors_.hasPosition() && right_motors_.hasPosition()) {
            ticks_left_ = (int)left_motors_.getPosition();
            ticks_right_ = (int)right_motors_.getPosition();
        }
        else {
            ticks_left_ = left_encoder_.get() ;
            ticks_right_ = right_encoder_.get()  ;
        }
        dist_left_ = ticks_left_ / ticks_per_meter_;
        dist_right_ = ticks_right_ / ticks_per_meter_;

        speed_left_ = (dist_left_ - last_dist_left_) / getDeltaTime() ;
        speed_right_ = (dist_right_ - last_dist_right_) / getDeltaTime() ;

        last_dist_left_ = dist_left_ ;
        last_dist_right_ = dist_right_ ;
        odometry_.update(getHeading(), dist_left_, dist_right_) ;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(speed_left_, speed_right_) ;
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(gyro_.getAngle()) ;
    }

    public double getWidth() {
        return width_ ;
    }

    public Pose2d getPose() {
        return odometry_.getPoseMeters() ;
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry_.resetPosition(pose, getHeading());
    }

    public void tankDrivePower(double left, double right) {
        left_motors_.set(left) ;
        right_motors_.set(right) ;
        drive_.feed() ;
    }

    private void resetEncoders() {
        if (left_motors_.hasPosition() && right_motors_.hasPosition()) {
            left_motors_.resetEncoder();
            right_motors_.resetEncoder();
        }
        else {
            left_encoder_.reset();
            right_encoder_.reset() ;
        }
    }

    private boolean createGyro() throws BadParameterTypeException, MissingParameterException {
        boolean ret = true ;
        String type = getSetting("hw:gyro").getString() ;
        if (type.equals("navx")) {
            gyro_ = new NavxGyro() ;
        }
        else {
            MessageLogger logger = getMessageLogger() ;
            logger.startMessage(MessageType.Error).add("in subsystem ").addQuoted(getName()).add(" the gyro type ") ;
            logger.addQuoted(type).add(" is an invalid value").endMessage() ;
        }

        return ret ;
    }


    private void attachHardware() throws BadMotorRequestException, MissingParameterException, BadParameterTypeException {
        MessageLogger logger = getMessageLogger() ;

        left_motors_ = getMotorFactory().createMotor("TankDriveLeft", "subsystems:" + getName() + ":hw:left:motors") ;
        right_motors_ = getMotorFactory().createMotor("TankDriveRight", "subsystems:" + getName() + ":hw:right:motors") ;

        if (left_motors_ == null || right_motors_ == null) {

            logger.startMessage(MessageType.Error, getLoggerID()) ;
            if (left_motors_ == null)
                logger.add("could not create left motors, ") ;
            if (right_motors_ == null)
                logger.add("could not create right motors") ;            
            logger.endMessage();
        }


        if (!left_motors_.hasPosition() || !right_motors_.hasPosition()) {
            int p1, p2 ;

            p1 = getSetting("hw:left:encoders:1").getInteger() ;
            p2 = getSetting("hw:left:encoders:2").getInteger() ;
            left_encoder_ = new Encoder(p1, p2) ;


            p1 = getSetting("hw:right:encoders:1").getInteger() ;
            p2 = getSetting("hw:right:encoders:2").getInteger() ;
            right_encoder_ = new Encoder(p1, p2) ;

            logger.startMessage(MessageType.Info, getLoggerID()) ;
            logger.add("TankDrive FPGA encoder indexes: ") ;
            logger.add("left", left_encoder_.getFPGAIndex()) ;
            logger.add("right", right_encoder_.getFPGAIndex()) ;
            logger.endMessage();
        }

        if (left_motors_.hasPosition() && right_motors_.hasPosition()) {
            left_motors_.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Frequent);
            right_motors_.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Frequent);
        }
    }    
}
