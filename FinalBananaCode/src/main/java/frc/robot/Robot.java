package frc.robot; 

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.loops.Looper;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.LED.RingIntakeState;
import frc.robot.subsystems.Superstructure.RobotState;

public class Robot extends TimedRobot {
  // Loopers
  private final Looper enabledLooper = new Looper(Constants.kLooperDt);
  private final Looper disabledLooper = new Looper(Constants.kLooperDt);
  
  // Subsystems
  private Superstructure superstructure;
  private Drive drive;
  private Arm arm ;
  private Shooter shooter;
  private Intake intake;
  private LED led ;

  // Controllers
  private XboxController controller;
  private Joystick joystickLeft;
  private Joystick joystickRight;
  private Joystick controlBoard;

  // States
  Boolean systemsHealthy;


  @Override
  public void robotInit() {
    try {
      drive = Drive.getInstance();
      arm = Arm.getInstance();
      intake = Intake.getInstance();
      shooter = Shooter.getInstance();
      led = LED.getInstance();
      superstructure = Superstructure.getInstance();

      // Enable brakes
      arm.enableArmBrake();
      drive.enableDriveBrake(); 

      // Set up controller or joystick 
      controller = new XboxController(0);
      joystickLeft = new Joystick(2);
      joystickRight = new Joystick(1);
      controlBoard = new Joystick(3);

      // Check health
      systemsHealthy = checkSystems();

      led.setState(led.new HealthNotifyState(systemsHealthy));

    } catch (Throwable t) {
      throw t;
    }
  }

  @Override
  public void robotPeriodic(){
    try {
      superstructure.execute();

      // Reset NavX
      if(joystickLeft.getRawButton(11)){
        drive.resetHeading();
      }

      // Set robot states
      if(controlBoard.getRawButtonPressed(1)){
        superstructure.setState(RobotState.DRIVE);
      }
      if(controlBoard.getRawButtonPressed(2)){
        superstructure.setState(RobotState.INTAKE);
      }
      if(controlBoard.getRawButtonPressed(3)){
        superstructure.setState(RobotState.SPEAKER);
      }
      if(controlBoard.getRawButtonPressed(4)){
        superstructure.setState(RobotState.AMP);
      }
      if(controlBoard.getRawButtonPressed(5)){
        superstructure.setState(RobotState.CHAIN);
      }

      if(controlBoard.getRawButtonPressed(11)){
        systemsHealthy = checkSystems();
        led.setState(led.new HealthNotifyState(systemsHealthy));
      }

      // Telemetry
      outputTelemetries();
    }
    catch (Throwable t) {
      throw t;
    }
  }

  @Override 
  public void autonomousInit(){
    led.setState(led.new AutonomousState());
  }

  @Override 
  public void autonomousPeriodic(){
    
  }

  @Override 
  public void teleopInit(){
    drive.enableDriveBrake();
    arm.setManualMode();
    drive.setDriveMode(false);
    intake.setIntakeState(IntakeState.OFF);
    led.setState(led.new TeleopState());
  }

  @Override
  public void teleopPeriodic(){
    try {
      // Drive Inputs
      double leftDrive = controller.getRawAxis(1) + joystickLeft.getY(); 
      double rightDrive = -controller.getRawAxis(5) + -joystickRight.getY();

      // Arm Inputs
      double armVelocity = controller.getRawAxis(2) - controller.getRawAxis(3);
      Boolean runIntake = controller.getRawButton(5) || joystickLeft.getRawButton(1);
      Boolean runShooter = controller.getRawButton(6)  || joystickRight.getRawButton(1);
      Boolean runIntakeBack = controller.getYButton() || joystickLeft.getRawButton(4) || controlBoard.getRawButton(6);

      // Run subsystems
      drive.tankDrive(leftDrive, rightDrive);
      
      // Arm Overrides
      if(controlBoard.getRawAxis(1) == 1){
        armVelocity = -0.5;
      } else if(controlBoard.getRawAxis(1)== -1){
        armVelocity = 0.5;
      }

      if(joystickLeft.getPOV() == 0){
        armVelocity = 0.75;
        if(superstructure.getState() == RobotState.CHAIN){
          armVelocity = 1;
        }
      } else if(joystickLeft.getPOV() == 180){
        armVelocity = -0.75;
        if(superstructure.getState() == RobotState.CHAIN){
          armVelocity = -1;
        }
      }

      if(joystickRight.getPOV() == 0){
        armVelocity = 0.5;
      } else if(joystickRight.getPOV() == 180){
        armVelocity = -0.5;
      }

      // Run Arm
      if(!controlBoard.getRawButton(12)){
        arm.armPeriodic(armVelocity);
      } else {
        arm.moveArm(0, true);
        arm.stop();
      }

      // Run Intake
      if(runIntake){
        intake.setIntakeState(IntakeState.INTAKING);
      } else if (runIntakeBack){
        intake.setIntakeState(IntakeState.EJECTING);
      } else {
      if(intake.getIntakeState() != IntakeState.OFF && 
        intake.getIntakeState() != IntakeState.AUTO_INTAKING &&
        intake.getIntakeState() != IntakeState.AUTO_EJECTING
        )
        intake.setIntakeState(IntakeState.OFF);
      } 

      // Run Shooter
      if(runShooter){
        shooter.shoot();
      } else {
        shooter.stop();
      }  

      
    }
    catch (Throwable t) {
      throw t;
    }
  }
  
  @Override
  public void disabledInit(){
    led.setState(led.new DisabledState());
  }

  @Override
  public void disabledPeriodic() {
   
  }


  // CUSTOM METHODS

  public void outputTelemetries(){
    drive.outputTelemetry(isDisabled());
    arm.outputTelemetry(isDisabled());
    shooter.outputTelemetry(isDisabled());
    intake.outputTelemetry(isDisabled());
  }

  public Boolean checkSystems(){
    return (
      drive.checkSystem() ||
      arm.checkSystem() ||
      shooter.checkSystem() ||
      intake.checkSystem() 
    );
  }
}