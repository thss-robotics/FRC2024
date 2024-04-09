package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Subsystem;

public class Shooter extends Subsystem {
    private static Shooter instance = null;

    // Motors
    private Spark lowerShooterMotor;
    private Spark upperShooterMotor;

    // States
    public enum ShooterState {
        OFF,
        SPOOLING_UP,
        SHOOTING,
        SHOOTING_AUTO,
    }

    private ShooterState currentState = ShooterState.OFF;
    private double shootSpeed = 1; 

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
        // Attach motors
        lowerShooterMotor = new Spark(0);
        upperShooterMotor = new Spark(1);
      
        // Configure motors
        upperShooterMotor.setInverted(false);
        lowerShooterMotor.setInverted(false);
    }

    public void setShooterState(ShooterState newState, double speed) {
        currentState = newState;
        updateMotors(speed);
    }

    public ShooterState getShooterState() {
        return currentState;  
    }

    private void updateMotors(double speed) {
        if (currentState == ShooterState.SHOOTING) {
            lowerShooterMotor.set(speed);
            upperShooterMotor.set(speed); 
        } else if (currentState == ShooterState.SHOOTING_AUTO) {
            lowerShooterMotor.set(speed);
            upperShooterMotor.set(speed); 
        } else {
            lowerShooterMotor.set(0);
            upperShooterMotor.set(0);
        }
    }

    public void shoot() {
        setShooterState(ShooterState.SHOOTING, shootSpeed);
    }

    @Override
    public void stop() {
        setShooterState(ShooterState.OFF, 0);
    }

    @Override
    public boolean checkSystem() {
        return  lowerShooterMotor.isAlive() &&
                upperShooterMotor.isAlive();
    }

    @Override
    public void outputTelemetry(boolean isRobotDisabled) {
        SmartDashboard.putString("Shooter State", currentState.toString());
        SmartDashboard.putNumber("Shooter Speed", shootSpeed * 100);
        
        SmartDashboard.putNumber("Lower Shooter Speed", lowerShooterMotor.get() * 100);
        SmartDashboard.putNumber("Upper Shooter Speed", upperShooterMotor.get() * 100);
    }
}
