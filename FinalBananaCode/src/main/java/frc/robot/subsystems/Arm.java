package frc.robot.subsystems;

import frc.lib.Subsystem;
import frc.robot.Constants;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends Subsystem{
    private static Arm instance = null;

    // Motors  
    private final WPI_TalonSRX leftLeader = new WPI_TalonSRX(5);
    private final WPI_TalonSRX rightFollower = new WPI_TalonSRX(6);

    // Encoder
    private static final SparkMaxAbsoluteEncoder.Type kAbsEncType = SparkMaxAbsoluteEncoder.Type.kDutyCycle;
    private AbsoluteEncoder armEncoder;

    // States
    public enum ArmState { MOVING, HOLDING, DISABLED } // Idle, Automatic, Brake stets, Error, Calibration, Lifting
    private ArmState currentState = ArmState.DISABLED;

    private SlewRateLimiter armFilter = new SlewRateLimiter(1.5);
    private Boolean brakeMode = true;

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    private Arm() {
        // Set default configuration
        leftLeader.configFactoryDefault();
        rightFollower.configFactoryDefault();

        // Set up right motor as a follower
        rightFollower.follow(leftLeader);
        rightFollower.setInverted(InvertType.OpposeMaster); 

        // Set brake state to brake by default
        leftLeader.setNeutralMode(NeutralMode.Brake);

        // Set up encoder
        armEncoder = Drive.getInstance().frontLeftMotor.getAbsoluteEncoder(kAbsEncType);

        currentState = ArmState.MOVING;
    }

    // Move at velocity from -1 to 1  
    public void moveArm(double velocity){
        if (currentState != ArmState.DISABLED) {
            velocity = armFilter.calculate(velocity);
            leftLeader.set(ControlMode.PercentOutput, velocity);
            currentState = ArmState.MOVING;
        }

        SmartDashboard.putNumber("Arm Velocity", armEncoder.getVelocity());
        SmartDashboard.putNumber("Arm Position Converstion Factor", armEncoder.getPositionConversionFactor());
        SmartDashboard.putNumber("Arm Angle", armEncoder.getPosition());
    }

    // STATE CONTROLS --------------------------------------------------->
    public ArmState getArmState(){
        return currentState;
    }


    // BRAKE CONTROLS --------------------------------------------------->

    public void toggleBrake() {
        brakeMode = !brakeMode;
        if(brakeMode){
            enableArmBrake();
        } else {
            disableArmBrake();
        }
    }

    public void enableArmBrake() {
        leftLeader.setNeutralMode(NeutralMode.Brake);
        rightFollower.setNeutralMode(NeutralMode.Brake);
    }

    public void disableArmBrake() {
        leftLeader.setNeutralMode(NeutralMode.Coast);
        rightFollower.setNeutralMode(NeutralMode.Coast);
    }


    // BASE CONTROLS --------------------------------------------------->

    @Override
    public void stop() {
        leftLeader.set(0);
        currentState = ArmState.HOLDING;
    }


    @Override
    public boolean checkSystem() {
        // Motor Checks
        boolean motorsOk =  leftLeader.getLastError() != ErrorCode.OK && leftLeader.isAlive() &&
                            rightFollower.getLastError() != ErrorCode.OK && rightFollower.isAlive();

        // Combine checks
        boolean systemOk = motorsOk;

        // Output status to SmartDashboard or console for debugging
        SmartDashboard.putBoolean("System Check - Motors OK", motorsOk);
        SmartDashboard.putBoolean("System Check - Overall Status", systemOk);

        return systemOk;
    }

    @Override
    public void outputTelemetry(boolean isRobotDisabled) {
        SmartDashboard.putBoolean("Arm Brakes", brakeMode);
        SmartDashboard.putNumber("Arm Output", leftLeader.getMotorOutputPercent() * 100);
        SmartDashboard.putString("Arm State", currentState.toString());

        SmartDashboard.putNumber("Arm Velocity", armEncoder.getVelocity());
        SmartDashboard.putNumber("Arm Position Converstion Factor", armEncoder.getPositionConversionFactor());
        SmartDashboard.putNumber("Arm Angle", armEncoder.getPosition());
    }
}