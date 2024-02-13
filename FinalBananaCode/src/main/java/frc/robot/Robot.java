package frc.robot; 
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Robot extends TimedRobot {
    WPI_TalonSRX leftLeader = new WPI_TalonSRX(5);
    WPI_TalonSRX rightFollower = new WPI_TalonSRX(6);

    XboxController controller;

    Boolean brakeMode = true;

    @Override
    public void robotInit() {
        // Set up controller or joystick 
        controller = new XboxController(0);

        // Initialize subsystems
        Arm.armInit();
        Drive.driveInit();
        Intake.intakeInit();
        Shooter.shooterInit();

        // Start camera server
        CameraServer.startAutomaticCapture();
    }

    @Override
    public void robotPeriodic(){
      // Set arm brakes
      if(controller.getAButton()){
        Arm.toggleArmBrake();
      }

      // Set drive brakes
      if(controller.getBButton()){
        Drive.toggleDriveBrake();
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
      Drive.setSpeedLimit(speedLimit);
    }


    @Override
    public void teleopPeriodic() {
      // Drive
      double leftDrive = controller.getLeftY();
      double rightDrive = controller.getRightY();
      double turn = controller.getLeftX();
        
      Drive.drive(leftDrive, rightDrive, turn);

      // Move arm
      double armVelocity = controller.getLeftTriggerAxis() - controller.getRightTriggerAxis();
      Arm.moveArm(armVelocity);

      // Toggle Intake
      if(controller.getXButton()){
        if(Intake.intakeRunning)
        {
          Intake.intakeOff();
        } else{
          Intake.intakeOn();
        }  
      }

      // Toggle Shooter
      if(controller.getYButton()){
        if(Shooter.shooting)
        {
          Shooter.shoot();
        } else{
          Shooter.driveShooterMotors(0);
        }  
      }

      // Set drive brakes
      if(controller.getXButton()){
        Intake.intakeOn();
      }
    }

    @Override 
    public void testInit(){
     
    }

    @Override
    public void testPeriodic(){
      // Move arm
      double armVelocity = controller.getLeftTriggerAxis() - controller.getRightTriggerAxis();
      Arm.moveArm(armVelocity);

      
    }

    @Override
    public void disabledInit() {
      Arm.disableArmBrake();
      Drive.disableDriveBrake();
    }
}