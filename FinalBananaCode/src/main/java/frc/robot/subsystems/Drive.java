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
    private static DifferentialDrive driveSystem;

    // IMU
    static AHRS navX;

    // Motors
    private static CANSparkMax frontLeftMotor;
    private static CANSparkMax frontRightMotor;
    private static CANSparkMax backLeftMotor;
    private static CANSparkMax backRightMotor;

    // Filters
    private static Boolean enableFilter = false;
    private static SlewRateLimiter filterLeft = new SlewRateLimiter(6);
    private static SlewRateLimiter filterRight = new SlewRateLimiter(6);
    private static SlewRateLimiter filterTurn = new SlewRateLimiter(6);

    // States
    private static double speedLimit = 1;
    private static Boolean brakeMode = true;


    public static void driveInit() {
        // Attach motors
        frontLeftMotor = new CANSparkMax(Constants.kBackLeftDriveId, MotorType.kBrushed);
        frontRightMotor = new CANSparkMax(Constants.kFrontRightDriveId, MotorType.kBrushed);
        backLeftMotor = new CANSparkMax(Constants.kBackLeftDriveId, MotorType.kBrushed);
        backRightMotor = new CANSparkMax(Constants.kBackRightDriveId, MotorType.kBrushed);

        // Configure motors
        backLeftMotor.follow(frontLeftMotor);
        backRightMotor.follow(frontRightMotor);
        
        frontLeftMotor.setIdleMode(IdleMode.kBrake);
        frontRightMotor.setIdleMode(IdleMode.kBrake);

        driveSystem = new DifferentialDrive(frontLeftMotor, frontRightMotor);

        
        // Attach IMU
        try {
            navX = new AHRS(SPI.Port.kMXP);
            navX.enableLogging(true);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
    }

    public static void drive(double leftDrive, double rightDrive, double turn) {
        leftDrive *= speedLimit;
        rightDrive *= speedLimit;
        turn *= speedLimit;
    
        if(enableFilter){
          leftDrive = filterLeft.calculate(leftDrive);
          rightDrive = filterRight.calculate(rightDrive);
          turn = filterTurn.calculate(turn);
        }
       
        driveSystem.tankDrive(leftDrive, -rightDrive);
    }


    // STATE CONTROLS --------------------------------------------------->

    public static void toggleDriveBrake() {
        brakeMode = !brakeMode;
        if(brakeMode){
            enableDriveBrake();
        } else {
            disableDriveBrake();
        }
    }

    public static void enableDriveBrake() {
        frontLeftMotor.setIdleMode(IdleMode.kBrake);
        frontRightMotor.setIdleMode(IdleMode.kBrake);
    }

    public static void disableDriveBrake() {
        frontLeftMotor.setIdleMode(IdleMode.kCoast);
        frontRightMotor.setIdleMode(IdleMode.kCoast);
    }

    public static void setSpeedLimit(double limit){
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
