package frc.robot; 

import java.time.YearMonth;

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

  // Auto
  private AutoManager autoManager;
  private AutoModeSelector autoModeSelector;
  private FollowTrajectoryCommand followTrajectoryCommand;

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
      
      // Auto
      autoManager = AutoManager.getInstance();
      autoModeSelector = new AutoModeSelector();

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
      if(controller.getAButtonPressed()){
        superstructure.setState(RobotState.INTAKE);
      }
      if(controller.getAButtonPressed()){
        superstructure.setState(RobotState.SPEAKER);
      }
      if(controller.getAButtonPressed()){
        superstructure.setState(RobotState.AMP);
      }/* 
      if(controlBoard.getRawButtonPressed(5)){
        superstructure.setState(RobotState.CHAIN);
      }

      if(controlBoard.getRawButtonPressed(11)){
        systemsHealthy = subsystemManager.checkSubsystems();
        led.setState(led.new HealthNotifyState(systemsHealthy));
      }
      */

      // Telemetry
      subsystemManager.outputToSmartDashboard(isDisabled());
    }
    catch (Throwable t) {
      throw t;
    }
  }

  @Override 
  public void autonomousInit(){
    led.setState(led.new AutonomousState());

    //autoModeSelector.getAutoModeForParams();

    var trajectory = autoManager.getTrajectory();
    followTrajectoryCommand = new FollowTrajectoryCommand(trajectory);
    followTrajectoryCommand.schedule();
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
     

      // Arm Inputs
      double armVelocity = 0;
      Boolean runIntake = false;
      Boolean runShooter = false;
      Boolean runIntakeBack = false;
      Boolean shootRing = false;

      // Adjust controls based on mode
      RobotState currentState = superstructure.getState();
    
      if(currentState == RobotState.DRIVE){
        runShooter = controller.getRightBumper();
        runIntake = controller.getLeftBumper();
        runIntakeBack = controller.getAButtonPressed();
      } else if(currentState == RobotState.INTAKE){
        runIntake = controller.getRightBumperPressed();
        runIntakeBack = controller.getLeftBumperPressed();
      } else if(currentState == RobotState.SPEAKER){
        runIntakeBack = controller.getLeftBumperPressed();
        shootRing = controller.getRightBumperPressed();
      } else if(currentState == RobotState.AMP){
        runIntakeBack = controller.getRightBumperPressed();
        shootRing = controller.getLeftBumperPressed();
      } 

      // Overrides
      runIntake = runIntake || controller.getRawButton(5);
      runIntakeBack = runIntakeBack || controller.getYButton();
      runShooter = runShooter || controller.getRawButton(6);
      armVelocity = controller.getRawAxis(2) - controller.getRawAxis(3);

    
   
     

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

    autoModeSelector.outputToSmartDashboard();
    autoModeSelector.updateModeCreator();
  }
}