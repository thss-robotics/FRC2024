package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;

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

        // Start camera server
        CameraServer.startAutomaticCapture();
    }

    @Override
    public void robotPeriodic(){
      
      // Brake Mode Testing  
    }


    @Override
    public void teleopPeriodic() {
      // Drive
      double leftDrive = controller.getLeftY();
      double rightDrive = controller.getRightY();
      double turn = controller.getLeftX();
        
      Drive.drive(leftDrive, rightDrive, turn);
    }

    @Override 
    public void testInit(){
     
    }

    @Override
    public void testPeriodic(){
      // Move arm
      double armVelocity = controller.getLeftTriggerAxis() - controller.getRightTriggerAxis();
      Arm.moveArm(armVelocity);

      // Set arm brakes
      if(controller.getAButton()){
        Arm.toggleBrake();
      }
    }

    @Override
    public void disabledInit() {
      
    }
}