package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Subsystem;

public class Intake extends Subsystem {
    private static Intake instance = null;

    // Intake Sensor
    DigitalInput intakeSensor = new DigitalInput(9);

    // Motors
    private CANSparkMax intakeMotor;

    // States
    public enum IntakeState {
        OFF,
        INTAKING,
        EJECTING
    }
    private IntakeState currentState = IntakeState.OFF;
    
    public Boolean intakeRunning = true;
    private static double INTAKE_SPEED = 1.0;
    private static double EJECT_SPEED = -0.5;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {
        // Attach motors
        intakeMotor = new CANSparkMax(7, MotorType.kBrushless);
        intakeMotor.setSmartCurrentLimit(60);
    }

    public void setIntakeState(IntakeState newState) {
        currentState = newState;
        updateMotor();
    }

    private void updateMotor() {
        switch (currentState) {
            case INTAKING:
                intakeMotor.set(INTAKE_SPEED);
                break;
            case EJECTING:
                intakeMotor.set(EJECT_SPEED);
                break;
            case OFF:
            default:
                intakeMotor.stopMotor();
                break;
        }
    }
      
    // BASE CONTROLS --------------------------------------------------->

    @Override
    public void stop() {
        intakeRunning = false;
        intakeMotor.stopMotor();
    }

    @Override
    public boolean checkSystem() {
        boolean motorOK = intakeMotor.getFaults() == 0;
        return motorOK;
    }

    @Override
    public void outputTelemetry(boolean isRobotDisabled) {
        SmartDashboard.putBoolean("Is Intaking", intakeSensor.get());
        SmartDashboard.putNumber("Intake Speed", intakeMotor.get() * 100);
        SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("Intake Speed", INTAKE_SPEED);
    }
}
