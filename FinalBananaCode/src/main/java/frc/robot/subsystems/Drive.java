package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Subsystem;
import frc.robot.Constants;

public class Drive extends Subsystem {
    // Drive System
    private DifferentialDrive driveSystem;

    // IMU
    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    // Motors
    private CANSparkMax frontLeftMotor;
    private CANSparkMax frontRightMotor;
    private CANSparkMax backLeftMotor;
    private CANSparkMax backRightMotor;

    // Filters
    private Boolean enableFilter = false;
    private SlewRateLimiter filterLeft = new SlewRateLimiter(6);
    private SlewRateLimiter filterRight = new SlewRateLimiter(6);
    private SlewRateLimiter filterTurn = new SlewRateLimiter(6);

    // States
    private double speedLimit = 1;
    private Boolean brakeMode = true;


    public void driveInit() {
        // Attach motors
        frontLeftMotor = new CANSparkMax(Constants.kFrontLeftDriveId, MotorType.kBrushed);
        frontRightMotor = new CANSparkMax(Constants.kFrontRightDriveId, MotorType.kBrushed);
        backLeftMotor = new CANSparkMax(Constants.kBackLeftDriveId, MotorType.kBrushed);
        backRightMotor = new CANSparkMax(Constants.kBackRightDriveId, MotorType.kBrushed);

        // Configure motors
        backLeftMotor.follow(frontLeftMotor);
        backRightMotor.follow(frontRightMotor);
        
        frontLeftMotor.setIdleMode(IdleMode.kBrake);
        frontRightMotor.setIdleMode(IdleMode.kBrake);

        driveSystem = new DifferentialDrive(frontLeftMotor, frontRightMotor);
    }

    public void arcadeDrive(double speed, double turn) {
        speed *= 1;
        turn *= 1;
     
        if(enableFilter){
          speed = filterLeft.calculate(speed);
          turn = filterTurn.calculate(turn);
        }
       

        SmartDashboard.putNumber("Left  Output", speed * 100);
        SmartDashboard.putNumber("Turn Output", turn * 100);

        driveSystem.arcadeDrive(speed, turn);
    }

    public void tankDrive(double leftDrive, double rightDrive) {
        leftDrive *= 1;
        rightDrive *= 1;
     
        if(enableFilter){
          leftDrive = filterLeft.calculate(leftDrive);
          rightDrive = filterRight.calculate(rightDrive);
        }

        SmartDashboard.putNumber("Left  Output", leftDrive * 100);
        SmartDashboard.putNumber("Right Output", rightDrive * 100);

        driveSystem.tankDrive(leftDrive, rightDrive);
    }


    // STATE CONTROLS --------------------------------------------------->

    public void toggleBrakes() {
        brakeMode = !brakeMode;
        if(brakeMode){
            enableDriveBrake();
        } else {
            disableDriveBrake();
        }
    }

    public void enableDriveBrake() {
        frontLeftMotor.setIdleMode(IdleMode.kBrake);
        frontRightMotor.setIdleMode(IdleMode.kBrake);
    }

    public void disableDriveBrake() {
        frontLeftMotor.setIdleMode(IdleMode.kCoast);
        frontRightMotor.setIdleMode(IdleMode.kCoast);
    }

    public void setSpeedLimit(double limit){
        speedLimit = limit;
    }

      
    // BASE CONTROLS --------------------------------------------------->

    @Override
    public void stop() {
       frontLeftMotor.stopMotor();;
       frontRightMotor.stopMotor();
    }

    @Override
    public boolean checkSystem() {
        return (
            frontLeftMotor.getLastError() != REVLibError.kError ||
            frontRightMotor.getLastError() != REVLibError.kError ||
            backLeftMotor.getLastError() != REVLibError.kError ||
            backRightMotor.getLastError() != REVLibError.kError 
        );
    }

    @Override
    public void outputTelemetry(boolean isRobotDisabled) {
        SmartDashboard.putNumber("Left Drivetrain Output", frontLeftMotor.get() * 100);
        SmartDashboard.putNumber("Right Drivetrain Output", frontRightMotor.get() * 100);
    }
}
