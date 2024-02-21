package frc.robot; 

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeState;

public class Robot extends TimedRobot {
    // Subsystems
    private Shooter shooter;
    private Drive drive;
    private Intake intake;
    private Arm arm ;

    // Controllers
    XboxController controller;
    Joystick controlBoard;
    Joystick joystickLeft;
    Joystick joystickRight;

    private Boolean tankDrive = true;
    private Boolean intakeDriveMode = false;

    @Override
    public void robotInit() {
      try {
        drive = Drive.getInstance();
        arm = Arm.getInstance();
        intake = Intake.getInstance();
        shooter = Shooter.getInstance();


        arm.enableArmBrake();
        drive.enableDriveBrake(); 
      } catch (Throwable t) {
        throw t;
      }

      // Set up controller or joystick 
      controller = new XboxController(0);
      joystickLeft = new Joystick(2);
      joystickRight = new Joystick(1);
      controlBoard = new Joystick(3);

      //Start camera server
      if(Constants.kEnableCameraServer){
        CameraServer.startAutomaticCapture();
      }
    }

    @Override
    public void robotPeriodic(){
      try {
        if(joystickLeft.getRawButton(7)){
          tankDrive = true;
        } else if(joystickLeft.getRawButton(8)){
          tankDrive = false;
        }

        if(joystickLeft.getRawButton(11)){
          drive.resetHeading();
        }

        if(joystickLeft.getRawButton(12)){
          intakeDriveMode = true;
        } else if(joystickLeft.getRawButton(11)){
          intakeDriveMode = false;
        }

      }
      catch (Throwable t) {
        throw t;
      }
    }

    @Override 
    public void teleopInit(){
      arm.enableArmBrake();
    }

    @Override
    public void teleopPeriodic(){
      try {
        // Drive Inputs
        double leftDrive = controller.getRawAxis(1) + (intakeDriveMode ? joystickLeft.getY() : -joystickRight.getY());
        double rightDrive = -controller.getRawAxis(5) + (intakeDriveMode ? -joystickRight.getY() : joystickLeft.getY());
        double turn = controller.getRawAxis(1) + joystickLeft.getX();


        // Arm Inputs
        double armVelocity = controller.getRawAxis(2) - controller.getRawAxis(3);
        Boolean runIntake = controller.getRawButton(5) || joystickLeft.getRawButton(1);
        Boolean runShooter = controller.getRawButton(6)  || joystickRight.getRawButton(1);
        Boolean runIntakeBack = controller.getYButton() || joystickLeft.getRawButton(4);
        
        // Run subsystems
        drive.tankDrive(leftDrive, rightDrive);
        

        if(controlBoard.getRawAxis(1) == 1){
          armVelocity = 1;
        } else if(controlBoard.getRawAxis(1)== -1){
          armVelocity = -1;
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

        arm.moveArm(armVelocity);

        // Run Intake
        if(runIntake){
          intake.setIntakeState(IntakeState.INTAKING);
        } else if (runIntakeBack){
          intake.setIntakeState(IntakeState.EJECTING);
        } else {
          intake.setIntakeState(IntakeState.OFF);
        } 

        // Run Shooter
        if(runShooter){
          shooter.shoot();
        } else {
          shooter.stop();
        }  

        // Diagnostics
        outputTelemetries();
      }
      catch (Throwable t) {
        throw t;
      }
    }

    @Override 
    public void testInit(){}

    @Override
    public void testPeriodic(){
      // Diagnostics
      outputTelemetries();
    }

    @Override
    public void disabledInit() {
      arm.enableArmBrake();
      drive.disableDriveBrake();
      intake.stop();
      shooter.stop();
    }

    @Override
    public void disabledPeriodic() {
    }

    public void outputTelemetries(){
      drive.outputTelemetry(isDisabled());
      arm.outputTelemetry(isDisabled());
      shooter.outputTelemetry(isDisabled());
      intake.outputTelemetry(isDisabled());
    }

    public void checkSystems(){
      SmartDashboard.putBoolean("Systems Are Healthy", (
        drive.checkSystem() ||
        arm.checkSystem() ||
        shooter.checkSystem() ||
        intake.checkSystem() 
      ));
    }
}