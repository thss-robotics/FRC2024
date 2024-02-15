package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Subsystem;

public class Intake extends Subsystem {
    private static Intake instance = null;

    // Motors
    private CANSparkMax intakeMotor;

    // States
    public Boolean intakeRunning = true;
    private static double intakeSpeed = 1;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {
        // Attach motors
        intakeMotor = new CANSparkMax(7, MotorType.kBrushless);
    }

    public void runIntakeMotor(double speed) {
        if(speed == 0 ){
            intakeRunning = false;
        } else {
            intakeRunning = true;
        }
        intakeMotor.set(speed);
    }


    // STATE CONTROLS --------------------------------------------------->

    public void intakeOn() {
        runIntakeMotor(intakeSpeed);
    }

     public void intakeOff() {
         runIntakeMotor(0);
    }
      
    // BASE CONTROLS --------------------------------------------------->

    @Override
    public void stop() {
        intakeRunning = false;
        intakeMotor.stopMotor();
    }

    @Override
    public boolean checkSystem() {
        return (
           true 
        );
    }
    @Override
    public void outputTelemetry(boolean isRobotDisabled) {
        SmartDashboard.putBoolean("Is Intaking", intakeRunning);
        SmartDashboard.putNumber("Intake Speed", intakeMotor.get() * 100);
    }
 
}
