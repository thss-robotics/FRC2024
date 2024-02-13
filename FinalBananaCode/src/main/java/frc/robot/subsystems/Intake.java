package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Subsystem;
import frc.robot.Constants;

public class Intake extends Subsystem {
    // Motors
    private static CANSparkMax intakeMotor;

    // States
    public static Boolean intakeRunning = true;
    private static double intakeSpeed = 0.05;



    public static void intakeInit() {
        // Attach motors
        intakeMotor = new CANSparkMax(7, MotorType.kBrushed);
    }

    public static void driveIntakeMotor(double speed) {
        intakeMotor.set(speed);
    }


    // STATE CONTROLS --------------------------------------------------->

    public static void intakeOn() {
        intakeRunning = true;
        intakeMotor.set(intakeSpeed);
    }

     public static void intakeOff() {
        intakeRunning = false;
        intakeMotor.set(0);
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
