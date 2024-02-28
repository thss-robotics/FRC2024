package frc.robot; 

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.loops.Looper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
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
      superstructure.superstructurePeriodic();

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
      double armVelocity = 0;
      Boolean runIntake = false;
      Boolean runShooter = false;
      Boolean runIntakeBack = false;
      Boolean shootRing = false;

      RobotState currentState = superstructure.getState();
    
      if(currentState == RobotState.DRIVE){
        runShooter = joystickRight.getTrigger();
        runIntake = joystickLeft.getTrigger();
        runIntakeBack = joystickLeft.getRawButton(4);
      } else if(currentState == RobotState.INTAKE){
        runIntake = joystickRight.getTrigger();
        runIntakeBack = joystickLeft.getTrigger();
      } else if(currentState == RobotState.SPEAKER){
        runIntakeBack = joystickLeft.getTrigger();
        shootRing = joystickRight.getTriggerPressed();
      } else if(currentState == RobotState.AMP){
        runIntakeBack = joystickLeft.getTrigger();
        shootRing = joystickRight.getTriggerPressed();
      } 


      // Overrides
      runIntake = runIntake || controller.getRawButton(5);
      runIntakeBack = runIntakeBack || controlBoard.getRawButton(6) || controller.getYButton();
      runShooter = runShooter || controller.getRawButton(6);
      armVelocity = controller.getRawAxis(2) - controller.getRawAxis(3);

      if(controlBoard.getRawAxis(1) == 1){
        armVelocity = -0.5;
      } else if(controlBoard.getRawAxis(1)== -1){
        armVelocity = 0.5;
      }

      if(joystickLeft.getPOV() == 0){
        armVelocity = 0.75;
      } else if(joystickLeft.getPOV() == 180){
        armVelocity = -0.75;
      }

      if(joystickRight.getPOV() == 0){
        armVelocity = 0.5;
      } else if(joystickRight.getPOV() == 180){
        armVelocity = -0.5;
      }

      if(currentState == RobotState.CHAIN){
        armVelocity += joystickLeft.getTrigger() ? -1: 0;
        armVelocity += joystickRight.getTrigger() ? 1 : 0;
      }
    

      // Run drive
      drive.tankDrive(leftDrive, rightDrive);
   
      // Run Arm if not disabled
      if(!controlBoard.getRawButton(12)){
        arm.armPeriodic(armVelocity);
      } else {
        arm.moveArm(0, true);
        arm.stop();
      }

      // Run Intake
      IntakeState currentIntakeState = intake.getIntakeState();
      if(runIntake){
        intake.setIntakeState(IntakeState.INTAKING);
      } else if (runIntakeBack){
        intake.setIntakeState(IntakeState.EJECTING);
      } else if( currentIntakeState == IntakeState.INTAKING || 
            currentIntakeState == IntakeState.EJECTING ){
          intake.setIntakeState(IntakeState.OFF);
      } 

      // Run Shooter
      if(runShooter){
        shooter.setShooterState(ShooterState.SHOOTING, 100);
      } else if (shooter.getShooterState() == ShooterState.SHOOTING){
        shooter.stop();
      }  

      // Fire ring
      if(shootRing){
        superstructure.shootRing();
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