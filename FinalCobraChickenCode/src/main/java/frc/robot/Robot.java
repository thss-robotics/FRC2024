package frc.robot; 

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.loops.Looper;
import frc.lib.util.AdvantageKit;
import frc.robot.auto.AutoManager;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotState;

public class Robot extends LoggedRobot {
  // Subsystems
  private Superstructure superstructure;
  private SubsystemManager subsystemManager;
  private Drive drive;
  private Arm arm ;
  private Shooter shooter;
  private Intake intake;
  private LED led;
  private Camera camera;

  // Controllers
  private XboxController controller;

  // States
  Boolean systemsHealthy;

  // Hardware
  PowerDistribution powerDistribution;


  @Override
  public void robotInit() {
    try {
      // Init logging
      AdvantageKit.startLog();
      AdvantageKit.recordMetadata();

      // Instantiate subsystems
      subsystemManager = SubsystemManager.getInstance();
      drive = Drive.getInstance();
      arm = Arm.getInstance();
      intake = Intake.getInstance();
      shooter = Shooter.getInstance();
      led = LED.getInstance();
      camera = Camera.getInstance();
      superstructure = Superstructure.getInstance();
      
     
      // Enable brakes
      arm.enableArmBrake();
      drive.enableDriveBrake(); 

      // Set up controller or joystick 
      controller = new XboxController(0);
    
      // Check health
      systemsHealthy = subsystemManager.checkSubsystems();

      led.setState(led.new HealthNotifyState(systemsHealthy));

    } catch (Throwable t) {
      throw t;
    }
  }

  @Override
  public void robotPeriodic(){
    try {
      superstructure.superstructurePeriodic();

      // Set robot states
      if(controller.getAButtonPressed()){
        superstructure.setState(RobotState.DRIVE);
      }
      if(controller.getBButtonPressed()){
        superstructure.setState(RobotState.INTAKE);
      }
      if(controller.getXButtonPressed()){
        superstructure.setState(RobotState.SPEAKER);
      }
      if(controller.getYButtonPressed()){
        superstructure.setState(RobotState.AMP);
      }

      if(controller.getRightBumper()){
        systemsHealthy = subsystemManager.checkSubsystems();
        led.setState(led.new HealthNotifyState(systemsHealthy));
      }

      if(controller.getStartButtonPressed()){
        camera.toggleCameras();
      }

      // Telemetry
      /* 
      subsystemManager.outputToSmartDashboard(isDisabled());
      drive.outputTelemetry(isDisabled());
      arm.outputTelemetry(isDisabled());
      intake.outputTelemetry(isDisabled());
      shooter.outputTelemetry(isDisabled());
      superstructure.outputTelemetry(isDisabled());
      camera.outputTelemetry(isDisabled());*/
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
      double leftDrive = controller.getRawAxis(1) + controller.getLeftY(); 
      double rightDrive = -controller.getRawAxis(5) + -controller.getRightY();

      // Arm Inputs
      double armVelocity = 0;
      Boolean runIntake = false;
      Boolean runShooter = false;
      Boolean runIntakeBack = false;
      Boolean shootRing = false;

      // Adjust controls based on mode
      RobotState currentState = superstructure.getState();
    
      /* 
      if(currentState == RobotState.DRIVE){
        runShooter = controller.getRightTriggerAxis()> 0.2;
        runIntake = controller.getLeftTriggerAxis() > 0.2;
        runIntakeBack = controller.getLeftBumperPressed();
      } else if(currentState == RobotState.INTAKE){
        runIntake =  controller.getLeftTriggerAxis() > 0.2;
        runIntakeBack = controller.getRightTriggerAxis()> 0.2;
      } else if(currentState == RobotState.SPEAKER){
        runIntakeBack = controller.getRightTriggerAxis()> 0.2;
        shootRing =  controller.getRightTriggerAxis()> 0.2;
      } else if(currentState == RobotState.AMP){
        runIntakeBack = controller.getRightTriggerAxis()> 0.2;
        shootRing =  controller.getRightTriggerAxis()> 0.2;
      } 

      if(controller.getPOV() == 0){
        armVelocity = 0.5;
      } else if(controller.getPOV() == 180){
        armVelocity = -0.5;
      }

      if(currentState == RobotState.CHAIN){
        armVelocity += controller.getRightTriggerAxis() > 0.2 ? -1: 0;
        armVelocity += controller.getRightTriggerAxis() > 0.2 ? 1 : 0;
      }
    
      */

      armVelocity = controller.getLeftTriggerAxis() - controller.getRightTriggerAxis();

      runIntake = controller.getLeftBumper();
      runShooter = controller.getRightBumper();
      runIntakeBack = controller.getBackButton();
      // Drive
      drive.tankDrive(leftDrive, rightDrive);
   
      // Run Arm if not stopped
      if(!controller.getLeftStickButton() && !controller.getRightStickButton()){
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
    subsystemManager.checkSubsystems();
  }

  @Override
  public void disabledPeriodic() {
    DriverStation.getAlliance();
  }
}