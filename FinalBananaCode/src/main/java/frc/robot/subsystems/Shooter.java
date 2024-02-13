package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Subsystem;


public class Shooter extends Subsystem{
    // Motors
    private Spark LowerShooterMotor;
    private Spark UpperShooterMotor;

    // Filters
    private SlewRateLimiter filter = new SlewRateLimiter(2);
  
    // States
    public Boolean shooterRunning = true;
    private double shootSpeed = 0.6;


    public void shooterInit() {
        // Attach motors
        LowerShooterMotor = new Spark(0);
        UpperShooterMotor = new Spark(1);
      
        // Configure motors
        LowerShooterMotor.addFollower(UpperShooterMotor);
        UpperShooterMotor.setInverted(false);
    }

    public void runShooterMotors(double speed) {
        // Update state
        if(speed == 0){
            shooterRunning = false;
        } else {
            shooterRunning = true;
        }

        // Set speed
        speed = filter.calculate(speed);
        LowerShooterMotor.set(speed);

    }

    // STATE CONTROLS --------------------------------------------------->

    public void shoot() {
        shooterRunning = true;
        runShooterMotors(shootSpeed);
    }

    // BASE CONTROLS --------------------------------------------------->

    @Override
    public void stop() {
        shooterRunning = false;
        runShooterMotors(0);
    }

    @Override
    public boolean checkSystem() {
        return (
           LowerShooterMotor.isAlive() ||
           UpperShooterMotor.isAlive()
        );
    }

    @Override
    public void outputTelemetry(boolean isRobotDisabled) {
        SmartDashboard.putBoolean("Is Shooting", shooterRunning);
        SmartDashboard.putNumber("Lower Shooter", LowerShooterMotor.get() * 100);
        SmartDashboard.putNumber("Upper Shooter", UpperShooterMotor.get() * 100);
        SmartDashboard.putNumber("Shooter Speed", shootSpeed * 100);
    }
    
}