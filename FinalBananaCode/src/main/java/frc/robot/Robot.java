package frc.robot; 

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Robot extends TimedRobot {
    // Subsystems
    Shooter shooter = new Shooter();
    Drive drive = new Drive();
    Intake intake = new Intake();
    Arm arm = new Arm();

    // Controllers
    XboxController controller;
    Joystick joystickLeft;
    Joystick joystickRight;

    // States
    Boolean brakeMode = true;
    Boolean controlSchemaArm = false;

    @Override
    public void robotInit() {
        // Set up controller or joystick 
        controller = new XboxController(0);

        // Initialize subsystems
        arm.armInit();
        drive.driveInit();
        intake.intakeInit();
        shooter.shooterInit();

        //Start camera server
        if(Constants.kEnableCameraServer){
          CameraServer.startAutomaticCapture();
        }
    }

    @Override
    public void robotPeriodic(){
      // Toggle Control Schema
      if(controller.getRawButton(9)){
        controlSchemaArm = !controlSchemaArm;
      }

      // Set drive speed limit
      double speedLimit;
      int povAngle = controller.getPOV();
        
      if (povAngle == 0) {
          speedLimit = 1; // Set speed to maximum if POV is 0
      } else if (povAngle > 0) {
          speedLimit = ((double)povAngle / 315.0) * (1.0 - 0.0) + 0.0; // Scale linearly between 0 and 315 to a speed scale of 0 to 1
      } else {
          speedLimit = 0; // Default or safety speed
      }

      drive.setSpeedLimit(speedLimit);
    }


    @Override
    public void teleopPeriodic(){
      // Drive Inputs
      double leftDrive = -controller.getRawAxis(0);
      double turn = controller.getRawAxis(1);

      // Arm Inputs
      double armVelocity = controller.getRawAxis(3);
      Boolean runIntake = controller.getRawButton(1);
      Boolean runShooter = controller.getRawButton(2);
        
      // Run subsystems
      drive.drive(leftDrive, 0, turn);
      arm.moveArm(armVelocity);

      // Run Intake
      if(runIntake && !intake.intakeRunning){
        intake.intakeOn();
      } else{
        intake.intakeOff();
      } 

      // Run Shooter
      if(runShooter || !shooter.shooterRunning){
        shooter.shoot();
      } else {
        shooter.stop();
      }  

      // Diagnostics
      outputTelemetries();
    }

    @Override 
    public void testInit(){}

    @Override
    public void testPeriodic(){
      shooter.runShooterMotors(controller.getRawAxis(1));
      intake.runIntakeMotor(controller.getRawAxis(3));

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
      if(controller.getRawButton(1)){
        arm.toggleBrake();
      } else if(controller.getRawButton(2)){
        drive.toggleBrakes();
      } else if(controller.getRawButton(5)){
        drive.toggleBrakes();
      } else if(controller.getRawButton(6)){
        arm.toggleBrake();
      }
    }

    public void outputStates(){
      SmartDashboard.putBoolean("Control Schema Alternate", controlSchemaArm);
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