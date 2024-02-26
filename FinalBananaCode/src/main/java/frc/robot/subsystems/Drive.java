package frc.robot.subsystems;

import frc.lib.Subsystem;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem {
    private static Drive instance = null;

    // Drive System
    private DifferentialDrive driveSystem;

    // IMU
    private final AHRS navX = new AHRS(SPI.Port.kMXP);
    private double lastHeadingError = 0.0;
    private final double kTurnCorrection = 0.1; 

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
    private Boolean reverseForwardsDirection = false;


    public static Drive getInstance() {
        if (instance == null) {
            instance = new Drive();
        }
        return instance;
    }

    private Drive() {
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

        // Set up tank/arcade drive system
        driveSystem = new DifferentialDrive(frontLeftMotor, frontRightMotor);

        // Reset IMU
        navX.reset();
    }

    public void tankDrive(double leftDrive, double rightDrive) {
        // Apply speed limits
        leftDrive *= speedLimit;
        rightDrive *= speedLimit;
     
        if(enableFilter){
          leftDrive = filterLeft.calculate(leftDrive);
          rightDrive = filterRight.calculate(rightDrive);
        }

        driveSystem.tankDrive(leftDrive, rightDrive, true);
    }

    public void setDriveMode() {
        setSpeedLimit(1.0); 
        reverseForwardsDirection = false;
    }

    public void setIntakeMode() {
        setSpeedLimit(0.75); 
        reverseForwardsDirection = true;
    }

    public void setSpeakerMode() {
        setSpeedLimit(0.6); 
        reverseForwardsDirection = false;
    }

    public void setAmpMode() {
        setSpeedLimit(0.6); 
        reverseForwardsDirection = false;
    }

    public void setChainMode() {
        setSpeedLimit(0.75);
        reverseForwardsDirection = true;
    }


    // DRIVE STATE CONTROLS --------------------------------------------------->

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

    public double getSpeedLimit() {
        return speedLimit;
    }

    public void setSpeedLimit(double limit) {
        if (limit > 0 && limit <= 1) {
            speedLimit = limit;
        } else {
            System.err.println("Invalid speed limit: " + limit);
        }
    }

    // IMU --------------------------------------------------->

    private double getHeadingError() {
        double currentError = -navX.getAngle(); // Assumes reset to zero is straight ahead
        double errorChange = currentError - lastHeadingError;
        lastHeadingError = currentError;
        return errorChange;
    }

    public void resetHeading() {
        navX.reset();
    }

      
    // SUBSYSTEM BASE --------------------------------------------------->

    @Override
    public void stop() {
        frontLeftMotor.stopMotor();
        frontRightMotor.stopMotor();
        backLeftMotor.stopMotor();
        backRightMotor.stopMotor();
    }

    @Override
    public boolean checkSystem() {
        // Motor Checks
        boolean motorsOk =  frontLeftMotor.getLastError() == REVLibError.kOk &&
                            frontRightMotor.getLastError() == REVLibError.kOk &&
                            backLeftMotor.getLastError() == REVLibError.kOk &&
                            backRightMotor.getLastError() == REVLibError.kOk;

        // IMU Check
        boolean navXOk = navX.isConnected();
        boolean navXMageneticDisturbance = navX.isMagneticDisturbance();

        // Combine checks
        boolean systemOk = motorsOk && navXOk && navXMageneticDisturbance;

        // Output status to SmartDashboard or console for debugging
        SmartDashboard.putBoolean("System Check - Motors OK", motorsOk);
        SmartDashboard.putBoolean("System Check - NavX OK", navXOk);
        SmartDashboard.putBoolean("System Check - NavX Magnetic Disturbance", navXMageneticDisturbance);
        SmartDashboard.putBoolean("System Check - Overall Status", systemOk);

        return systemOk;
    }

    @Override
    public void outputTelemetry(boolean disabled) {
        // Drivetrain outputs
        SmartDashboard.putNumber("Left Drivetrain Output", frontLeftMotor.get() * 100);
        SmartDashboard.putNumber("Right Drivetrain Output", frontRightMotor.get() * 100);

        // IMU (navX) data
        SmartDashboard.putNumber("NavX Yaw", navX.getYaw());
        SmartDashboard.putNumber("NavX Pitch", navX.getPitch());
        SmartDashboard.putNumber("NavX Roll", navX.getRoll());
        SmartDashboard.putNumber("NavX Total Angle", navX.getAngle());
        SmartDashboard.putBoolean("NavX Is Calibrating", navX.isCalibrating());

        // Motor temperatures and current
        SmartDashboard.putNumber("Front Left Motor Temperature", frontLeftMotor.getMotorTemperature());
        SmartDashboard.putNumber("Front Right Motor Temperature", frontRightMotor.getMotorTemperature());
        SmartDashboard.putNumber("Back Left Motor Temperature", backLeftMotor.getMotorTemperature());
        SmartDashboard.putNumber("Back Right Motor Temperature", backRightMotor.getMotorTemperature());

        SmartDashboard.putNumber("Front Left Motor Current", frontLeftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Front Right Motor Current", frontRightMotor.getOutputCurrent());
        SmartDashboard.putNumber("Back Left Motor Current", backLeftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Back Right Motor Current", backRightMotor.getOutputCurrent());

        // Additional status
        SmartDashboard.putBoolean("Brake Mode Enabled", brakeMode);
        SmartDashboard.putNumber("Speed Limit", speedLimit);
    }
}
