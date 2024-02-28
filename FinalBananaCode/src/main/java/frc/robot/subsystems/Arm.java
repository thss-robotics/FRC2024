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
import edu.wpi.first.wpilibj.DigitalInput;
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

    private SlewRateLimiter armFilter = new SlewRateLimiter(1.8);
    private Boolean brakeMode = true;

    // PID
    private PIDController pidController;
    private double targetPosition;
    private final double minEffectivePower = 0.3;

    // States
    private boolean movingToDrive = false;

    // Limit Switch
    DigitalInput armLimitSwitch = new DigitalInput(5);


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
        armEncoder.setPositionOffset(0.846);

        currentState = ArmState.MOVING;
        currentPosition = ArmPosition.DRIVE;

        // PID 
        pidController = new PIDController(6, 0.05, 0.2);
        pidController.setTolerance(0.005);
        pidController.setIntegratorRange(-0.4, 0.4);
    }

    // Move at velocity from -1 to 1  
    public void moveArm(double velocity, boolean safetyEnabled){
        double armPosition = getArmPosition();

        // Slow zone
        if( (velocity < 0 && armPosition < 0.12) ||
            (velocity > 0 && armPosition > 0.40)
            ) {
            if (safetyEnabled){
                velocity *= 0.5;
            }
            if (Math.abs(velocity) > 0.25 && Superstructure.RobotState.CHAIN != Superstructure.getInstance().getState()){
                velocity = Math.copySign(0.25, velocity);
            }
        } 

        velocity = armFilter.calculate(velocity);

        // Safety Cutoff CANNOT BE DISABLEd
        if( (velocity < 0 && (!armLimitSwitch.get() || armPosition < 0.01) ) || 
            (velocity > 0 && armPosition > 0.47)){
            velocity = 0;
        }
        
        if (currentState != ArmState.DISABLED) {
            leftLeader.set(ControlMode.PercentOutput, velocity);
            currentState = ArmState.MOVING;
        }
    }
    

    public void moveArmToPosition(double absolutePosition){
        targetPosition = absolutePosition;
    }

    public void armPeriodic(double velocity) {
        SmartDashboard.putString("Arm current Position", currentPosition.toString());
        SmartDashboard.putBoolean("Arm moving to drive", movingToDrive);

        
        if(velocity < 0.25 && (currentPosition != ArmPosition.DRIVE || movingToDrive)){
            // Check the current arm position and adjust if necessary
            double currentPosition = getArmPosition(); // Gets the current encoder position
            if(targetPosition < currentPosition){
                pidController.setP(5);
            } else {
                pidController.setP(8);
            }
            double pidOutput = pidController.calculate(currentPosition, targetPosition);

                if (Math.abs(pidOutput) > 0 && Math.abs(pidOutput) < minEffectivePower) {
                    // Adjust the pidOutput to at least the minimum effective power
                    pidOutput = Math.copySign(minEffectivePower, pidOutput);
                }
                
            if (!pidController.atSetpoint()) { // Check if we're at the target position
                moveArm(pidOutput, false);
            } else {
                moveArm(0, true);
                movingToDrive = false;
            }   
        }
        else {
           
            moveArm(velocity, true);
            movingToDrive = false;
            currentPosition = ArmPosition.DRIVE;
        }   
    }

    public void setManualMode() {
        currentPosition = ArmPosition.DRIVE;
        movingToDrive = false;
    }

    public void moveToDrivePosition() {
        moveArmToPosition(Constants.kArmDriveAbsolutePosition);
        currentPosition = ArmPosition.DRIVE;
        movingToDrive = true;
    }
    
    public void moveToIntakePosition() {
        moveArmToPosition(Constants.kArmIntakeAbsolutePosition);
        currentPosition = ArmPosition.INTAKE;
    }
    
    public void moveToSpeakerPosition() {
        moveArmToPosition(Constants.kArmSpeakerAbsolutePosition);
        currentPosition = ArmPosition.SPEAKER;
    }
    
    public void moveToAmpPosition() {
        moveArmToPosition(Constants.kArmAmpAbsolutePosition);
        currentPosition = ArmPosition.AMP;
    }
    
    public void moveToChainPosition() {
        moveArmToPosition(Constants.kArmChainAbsolutePosition);
        currentPosition = ArmPosition.CHAIN;
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

        // Limit switch
        SmartDashboard.putBoolean("Arm Limit Switch", armLimitSwitch.get());

        // Encoder
        SmartDashboard.putBoolean("Encoder Connected", armEncoder.isConnected());
        SmartDashboard.putNumber("Arm Position", getArmPosition());
        SmartDashboard.putNumber("Arm Error", pidController.getPositionError());
    }
}