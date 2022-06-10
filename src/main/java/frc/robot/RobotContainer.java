// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.xero1425.base.XeroRobot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TankDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot class
  private XeroRobot robot_ ;

  // The subsystems
  private TankDriveSubsystem tankdrive_ ;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. 
   * @throws Exception
   */
  public RobotContainer(XeroRobot robot) throws Exception {
    robot_ = robot ;

    // Create the robot subsystems
    createSubsystems(); 

    // Configure the button bindings
    configureButtonBindings();
  }

  public TankDriveSubsystem getTankDrive() {
    return tankdrive_ ;
  }

  private void createSubsystems() throws Exception {
    tankdrive_ = new TankDriveSubsystem(robot_, "tankdrive") ;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
