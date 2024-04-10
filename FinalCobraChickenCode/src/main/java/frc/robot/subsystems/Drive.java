package frc.robot.subsystems;

import frc.lib.Subsystem;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem  {
    private static Drive instance = null;

    // Drive System and Odometry
    private DifferentialDrive driveSystem;
    private DifferentialDriveOdometry odometry;
    private DifferentialDriveKinematics kinematics =
        new DifferentialDriveKinematics(0.660);
    private Field2d field = new Field2d();

    // IMU
    private final AHRS navX = new AHRS(SPI.Port.kMXP);
    private double lastHeadingError = 0.0;
    private final double kTurnCorrection = 0.1; 

    // Motors
    private CANSparkMax frontLeftMotor;
    private CANSparkMax frontRightMotor;
    private CANSparkMax backLeftMotor;
    private CANSparkMax backRightMotor;

    // Encoders 
    private Encoder leftEncoder;
    private Encoder rightEncoder;

    // Filters
    private Boolean enableFilter = false;
    private SlewRateLimiter filterLeft = new SlewRateLimiter(6);
    private SlewRateLimiter filterRight = new SlewRateLimiter(6);
    private SlewRateLimiter filterTurn = new SlewRateLimiter(1);

    // States
    private double speedLimit = 0.7;
    private Boolean brakeMode = true;
    private Boolean reverseForwardsDirection = false;

    // Gyro Stabilization
    private boolean gyroStabilizationEnabled = false;
    private final double kPgyroStraight = 0.1; // Proportional gain for straight driving
    private final double straightDriveThreshold = 0.3; // Threshold to consider inputs as intending straight drive

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

        // Initalize encoders
        leftEncoder = new Encoder(1, 2);
        rightEncoder = new Encoder(3, 4);

        // Configure encoders
        leftEncoder.setReverseDirection(true);
        leftEncoder.reset();
        rightEncoder.reset();
        rightEncoder.setDistancePerPulse(0.0151993);
        leftEncoder.setDistancePerPulse(0.0151993);

        // Configure Odometry
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), getLeftDistance(), getRightDistance(), new Pose2d(0,0 , new Rotation2d()));

        // Set up tank/arcade drive system
        driveSystem = new DifferentialDrive(frontLeftMotor, frontRightMotor);

        // Reset IMU
        navX.reset();
    }

    // DRIVES

    public void tankDrive(double leftInput, double rightInput) {
        // Exponential scaling parameters
        double base = 2.0; 
    
        // Apply exponential scaling for finer control at low speeds
        double leftDrive = Math.signum(leftInput) * (Math.pow(base, Math.abs(leftInput)) - 1) / (base - 1);
        double rightDrive = Math.signum(rightInput) * (Math.pow(base, Math.abs(rightInput)) - 1) / (base - 1);
    
        // Apply speed limits
        leftDrive *= speedLimit;
        rightDrive *= speedLimit;
    
        // Existing logic for reverse direction and filters
        if (reverseForwardsDirection) {
            double temp = rightDrive;
            rightDrive = leftDrive;
            leftDrive = temp;
        }
    
        if (enableFilter) {
            leftDrive = filterLeft.calculate(leftDrive);
            rightDrive = filterRight.calculate(rightDrive);
        }
    
        // Existing logic for gyro stabilization
        boolean isStraightDrive = Math.abs(leftDrive + rightDrive) < straightDriveThreshold;
        boolean minimumStabilizeSpeed = (Math.abs(leftDrive) + Math.abs(rightDrive)) > 0.4;
        if (gyroStabilizationEnabled && isStraightDrive && minimumStabilizeSpeed) {
            double headingError = getHeadingError();
            double correction = kPgyroStraight * headingError;
            leftDrive -= correction;
            rightDrive -= correction;
        }
    
        // Drive the robot
        driveSystem.tankDrive(leftDrive, rightDrive, true);
    }
    
    public void drive(ChassisSpeeds chassisSpeeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        wheelSpeeds.desaturate(0.5);
        driveSystem.tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }

    // STATE SETTERS

    public void setDriveMode(Boolean reverse) {
        setSpeedLimit(1.0); 
        reverseForwardsDirection = reverse;
        gyroStabilizationEnabled = true;
    }

    public void setIntakeMode() {
        setSpeedLimit(0.75); 
        reverseForwardsDirection = true;
        gyroStabilizationEnabled = false;
    }

    public void setSpeakerMode() {
        setSpeedLimit(0.75); 
        reverseForwardsDirection = false;
        gyroStabilizationEnabled = false;
    }

    public void setAmpMode() {
        setSpeedLimit(0.6); 
        reverseForwardsDirection = false;
        gyroStabilizationEnabled = false;
    }

    public void setChainMode() {
        setSpeedLimit(0.75);
        reverseForwardsDirection = true;
        gyroStabilizationEnabled = false;
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

    public void toggleGyroStabilization() {
        gyroStabilizationEnabled = !gyroStabilizationEnabled;
    }

    // ODOMETRY

    public double getLeftDistance() {
        return leftEncoder.getDistance() / 600 / 5.5 * 47.8778 /1.5;
    }

    public double getRightDistance() {
        return rightEncoder.getDistance() / 600 / 5.5 * 47.8778 /1.5;
    }
    
    public void updateOdometry() {
        odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftDistance(), getRightDistance());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    // Resets the robot's odometry to the specified pose
    public void resetPose(Pose2d pose) {
        leftEncoder.reset();
        rightEncoder.reset();
        odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), new DifferentialDriveWheelPositions(getLeftDistance(), getRightDistance()), pose);
    }

    // Returns the current wheel speeds in a robot-relative manner
    public DifferentialDriveWheelSpeeds getRobotRelativeSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    private double getHeading() {
        return Math.IEEEremainder(navX.getAngle(), 360);
    }

    public ChassisSpeeds getCurrentSpeeds(){
        DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
        return kinematics.toChassisSpeeds(speeds);
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

    //@Override
    public void stop() {
        frontLeftMotor.stopMotor();
        frontRightMotor.stopMotor();
        backLeftMotor.stopMotor();
        backRightMotor.stopMotor();
    }

    //@Override
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

    // @Override
    public void outputTelemetry(boolean disabled) {
        // Drivetrain outputs
        SmartDashboard.putNumber("Left Drivetrain Output", -frontLeftMotor.get() * 100);
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


        // Encoder
        SmartDashboard.putNumber("Left Drive Encoder", getLeftDistance());
        SmartDashboard.putNumber("Right Drive Encoder", getRightDistance());
   
        // Additional status
        SmartDashboard.putBoolean("Brake Mode Enabled", brakeMode);
        SmartDashboard.putNumber("Speed Limit", speedLimit);
        SmartDashboard.putBoolean("Reverse Direction", reverseForwardsDirection);
        SmartDashboard.putBoolean("Gyro Assist Enabled", gyroStabilizationEnabled);
    }
}
