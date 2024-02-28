package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Subsystem;
import frc.robot.subsystems.LED.RingIntakeState;
import frc.robot.subsystems.Superstructure.RobotState;

public class Intake extends Subsystem {
    private static Intake instance = null;

    // Intake Sensor
    DigitalInput intakeSensor = new DigitalInput(9);

    // Motors
    private CANSparkMax intakeMotor;

    // LED
    private LED led = LED.getInstance();

    // States
    public enum IntakeState {
        OFF,
        AUTO_EJECTING,
        AUTO_INTAKING,
        INTAKING,
        EJECTING,
        SHOOTING,
    }

    public enum RingState {
        NONE,
        LOADED
    }

    private IntakeState currentState = IntakeState.OFF;
    private RingState ringState = RingState.NONE;
    
    public Boolean intakeRunning = true;
    private static double INTAKE_SPEED = 1.0;
    private static double EJECT_SPEED = -1.0;
    private static double AUTO_INTAKE_SPEED = 1.0;
    private static double AUTO_EJECT_SPEED = -0.5;
    private static double SHOOT_SPEED = 1;

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

    public void intakePeriodic(){
        updateMotor();
    }

    public void setIntakeState(IntakeState newState) {
        currentState = newState;
        updateMotor();
    }

    public IntakeState getIntakeState() {
        return currentState;
    }

    private void updateMotor() {
        switch (currentState) {
            case INTAKING:
                intakeMotor.set(INTAKE_SPEED);
                intakeMotor.setSmartCurrentLimit(60);
                break;
            case AUTO_INTAKING:
                intakeMotor.set(AUTO_INTAKE_SPEED);
                if(!intakeSensor.get()){
                    Superstructure.getInstance().setState(RobotState.DRIVE);
                    setIntakeState(IntakeState.AUTO_EJECTING);
                    led.setState(led.new RingIntakeState());
                }
                break;
            case AUTO_EJECTING:
                intakeMotor.setSmartCurrentLimit(60);
                intakeMotor.set(AUTO_EJECT_SPEED);
                if(intakeSensor.get()){
                    setIntakeState(IntakeState.OFF);
                    ringState = RingState.LOADED;
                }
                break;
            case EJECTING:
                intakeMotor.setSmartCurrentLimit(100);
                intakeMotor.set(EJECT_SPEED);
                break;
            case SHOOTING:
                intakeMotor.setSmartCurrentLimit(100);
                intakeMotor.set(SHOOT_SPEED);
                if(!intakeSensor.get()){
                    ringState = RingState.NONE;
                }
            case OFF:
                intakeMotor.setSmartCurrentLimit(60);
            default:
                intakeMotor.stopMotor();
                break;
        }
    }

    public Boolean ringLoaded(){
        Boolean ringInIntake = !intakeSensor.get();
        Boolean ringLoadedAndSet = ringState == RingState.LOADED;
        return ringInIntake || ringLoadedAndSet;
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
        SmartDashboard.putString("Intake State", currentState.toString());
    }
}
