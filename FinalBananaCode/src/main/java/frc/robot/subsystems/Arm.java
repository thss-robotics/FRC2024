package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Subsystem;


public class Arm extends Subsystem{
    // Motors  
    private final WPI_TalonSRX leftLeader = new WPI_TalonSRX(5);
    private final WPI_TalonSRX rightFollower = new WPI_TalonSRX(6);

    // States
    private Boolean brakeMode = true;

    // Connect hardware and set configurations
    public void armInit() {
        // Set default configuration
        leftLeader.configFactoryDefault();
        rightFollower.configFactoryDefault();

        // Set up right motor as a follower
        rightFollower.follow(leftLeader);
        rightFollower.setInverted(InvertType.OpposeMaster); 

        // Set brake state to brake by default
        leftLeader.setNeutralMode(NeutralMode.Brake);
    }

    // Move at velocity from -1 to 1  
    public void moveArm(double velocity){
        leftLeader.set(ControlMode.PercentOutput, velocity);
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
    }

    @Override
    public boolean checkSystem() {
        return leftLeader.isAlive();
    }

    @Override
    public void outputTelemetry(boolean isRobotDisabled) {
       SmartDashboard.putBoolean("Arm Brakes", brakeMode);
       SmartDashboard.putNumber("Arm Output", leftLeader.get() * 100);
    }

    
}