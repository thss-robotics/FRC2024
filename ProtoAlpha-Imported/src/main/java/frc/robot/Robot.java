/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PowerSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  Command m_autonomousCommand;
	SendableChooser<Command> autonChooser = new SendableChooser<Command>(); // Create a chooser to select an autonomous command

  public static final GenericHID controller = new GenericHID(Constants.CONTROLLER_USB_PORT_ID); // Instantiate our controller at the specified USB port

  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem(); // Drivetrain subsystem
  public static final PowerSubsystem m_powerSubsystem = new PowerSubsystem(); // Power subsystem for interacting with the Rev PDH
  public static final VisionSubsystem m_visionSubsystem = new VisionSubsystem(); // Subsystem for interacting with Photonvision
  boolean launchStarted = false;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // // Starts recording to data log
    // DataLogManager.start();

    // // Record both DS control and joystick data
    // DriverStation.startDataLog(DataLogManager.getLog());

    configureButtonBindings(); // Bind our commands to physical buttons on a controller

    m_driveSubsystem.setDefaultCommand(new DriveCommand());

    // Zero the gyro and reset encoders
    m_driveSubsystem.zeroGyro();
    m_driveSubsystem.resetEncoders();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods. This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Gyroscope Pitch", m_driveSubsystem.getPitch());
    SmartDashboard.putNumber("Gyroscope Yaw", m_driveSubsystem.getYaw());
    SmartDashboard.putNumber("Gyroscope Roll", m_driveSubsystem.getRoll());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // Stops recording to data log
    DataLogManager.stop();
    DataLogManager.log("======================================= DISABLED! ================================");
  }

  /** This function is called continuously after the robot enters Disabled mode. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // Starts recording to data log
    DataLogManager.start();

    System.out.println("AUTONOMOUS MODE STARTED");

    m_autonomousCommand = autonChooser.getSelected();
    
    // Zero the gyro and reset encoders
    m_driveSubsystem.zeroGyro();
    m_driveSubsystem.resetEncoders();

    // schedule the selected autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // Set the LED pattern for autonomous
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // Starts recording to data log
    DataLogManager.start();

    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this if statement or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Zero the gyro and reset encoders
    m_driveSubsystem.zeroGyro();
    m_driveSubsystem.resetEncoders();

    // Set the LED pattern for teleop

    SmartDashboard.putBoolean("Trigger Triggered", false);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Log controller inputs
    SmartDashboard.putNumber("Controller: Right Trigger", controller.getRawAxis(Constants.RIGHT_TRIGGER_AXIS));
    SmartDashboard.putNumber("Controller: Left Trigger", controller.getRawAxis(Constants.LEFT_TRIGGER_AXIS));
    SmartDashboard.putBoolean("Controller: Right Bumper", controller.getRawButton(Constants.RIGHT_BUMPER));
    SmartDashboard.putBoolean("Controller: Left Bumper", controller.getRawButton(Constants.LEFT_BUMPER));
    SmartDashboard.putBoolean("Controller: X Button", controller.getRawButton(Constants.X_BUTTON));
    SmartDashboard.putBoolean("Controller: Y Button", controller.getRawButton(Constants.Y_BUTTON));
    SmartDashboard.putBoolean("Controller: B Button", controller.getRawButton(Constants.B_BUTTON));
    SmartDashboard.putBoolean("Controller: A Button", controller.getRawButton(Constants.A_BUTTON));
    SmartDashboard.putNumber("Controller: Left Joystick X Axis", controller.getRawAxis(Constants.LEFT_HORIZONTAL_JOYSTICK_AXIS));
    SmartDashboard.putNumber("Controller: Left Joystick Y Axis", controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS));
    SmartDashboard.putNumber("Controller: Right Joystick X Axis", controller.getRawAxis(Constants.RIGHT_HORIZONTAL_JOYSTICK_AXIS));
    SmartDashboard.putNumber("Controller: Right Joystick Y Axis", controller.getRawAxis(Constants.RIGHT_VERTICAL_JOYSTICK_AXIS));
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or onse of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} 
   * or {@link XboxController}), and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.Trigger}.
   */
  private void configureButtonBindings() {
    // Drivetrain Controls //

  }
}