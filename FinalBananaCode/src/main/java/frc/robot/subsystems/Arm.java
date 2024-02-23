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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends Subsystem{
    private static Arm instance = null;

    // Motors  
    private final WPI_TalonSRX leftLeader = new WPI_TalonSRX(5);
    private final WPI_TalonSRX rightFollower = new WPI_TalonSRX(6);

    // Encoder
    private DutyCycleEncoder  armEncoder;

    // States
    public enum ArmState { MOVING, HOLDING, DISABLED } 
    private ArmState currentState = ArmState.DISABLED;

    public enum ArmPosition { DRIVE, INTAKE, SPEAKER, AMP, CHAIN } 
    private ArmPosition currentPosition = ArmPosition.DRIVE;

    private SlewRateLimiter armFilter = new SlewRateLimiter(1.5);
    private Boolean brakeMode = true;

    // PID
    private PIDController pidController;
    private double targetPosition;
    private final double minEffectivePower = 0.3;

     

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
        armEncoder = new DutyCycleEncoder(0);
        armEncoder.setPositionOffset(0.688);

        currentState = ArmState.MOVING;
        currentPosition = ArmPosition.DRIVE;

        // PID 
        pidController = new PIDController(6, 0.2, 0.2);
        pidController.setTolerance(0.005);
        pidController.setIntegratorRange(-0.4, 0.4);
    }

    // Move at velocity from -1 to 1  
    public void moveArm(double velocity){
        if (currentState != ArmState.DISABLED) {
            if(velocity > 0.7){
                velocity = 0.7;
            }
            if(velocity < -0.7){
                velocity = -0.7;
            }
            velocity = armFilter.calculate(velocity);
            leftLeader.set(ControlMode.PercentOutput, velocity);
            currentState = ArmState.MOVING;
        }
    }

    public void moveArmToPosition(double absolutePosition){
        targetPosition = absolutePosition;
    }

    public void armPeriodic() {
        // Check the current arm position and adjust if necessary
        double currentPosition = getArmPosition(); // Gets the current encoder position

        double pidOutput = pidController.calculate(currentPosition, targetPosition);

            if (Math.abs(pidOutput) > 0 && Math.abs(pidOutput) < minEffectivePower) {
                // Adjust the pidOutput to at least the minimum effective power
                pidOutput = Math.copySign(minEffectivePower, pidOutput);
            }

        if (!pidController.atSetpoint()) { // Check if we're at the target position
            moveArm(pidOutput);
        } else {
            moveArm(0);
        }
    }

    public void moveToDrivePosition() {
        moveArmToPosition(Constants.kArmDriveAbsolutePosition);
    }
    
    public void moveToIntakePosition() {
        moveArmToPosition(Constants.kArmIntakeAbsolutePosition);
    }
    
    public void moveToSpeakerPosition() {
        moveArmToPosition(Constants.kArmSpeakerAbsolutePosition);
    }
    
    public void moveToAmpPosition() {
        moveArmToPosition(Constants.kArmAmpAbsolutePosition);
    }
    
    public void moveToChainPosition() {
        moveArmToPosition(Constants.kArmChainAbsolutePosition);
    }
    
    

    public double getArmPosition(){
        double relativeArmPosition = armEncoder.getAbsolutePosition() - armEncoder.getPositionOffset();
        if(relativeArmPosition < 0){
            relativeArmPosition = 1 + relativeArmPosition;
        }

        if(relativeArmPosition > 0.9){
            relativeArmPosition = 0;
        }

        return relativeArmPosition;
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

        SmartDashboard.putNumber("Arm Target Position", targetPosition);

        // Encoder
        SmartDashboard.putBoolean("Encoder Connected", armEncoder.isConnected());
        SmartDashboard.putNumber("Arm Position", getArmPosition());
        SmartDashboard.putNumber("Arm Error", pidController.getPositionError());
    }
}