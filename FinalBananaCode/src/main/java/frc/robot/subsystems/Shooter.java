package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Subsystem;


public class Shooter extends Subsystem{
    // Motors
    private static Spark LowerShooterMotor;
    private static Spark UpperShooterMotor;

    // Filters
    private static SlewRateLimiter filter = new SlewRateLimiter(2);
  
    // States
    public static Boolean shooting = true;
    private static double shootSpeed = 0.2;



    public static void shooterInit() {
        // Attach motors
        LowerShooterMotor = new Spark(0);
        UpperShooterMotor = new Spark(1);
      
        // Configure motors
        LowerShooterMotor.addFollower(UpperShooterMotor);
        UpperShooterMotor.setInverted(true);
    }

    public static void driveShooterMotors(double speed) {
        speed = filter.calculate(speed);
        LowerShooterMotor.set(speed);
    }


    // STATE CONTROLS --------------------------------------------------->

    public static void shoot() {
        shooting = true;
        LowerShooterMotor.set(shootSpeed);
    }
      
    // BASE CONTROLS --------------------------------------------------->

    @Override
    public void stop() {
        shooting = false;
        LowerShooterMotor.stopMotor();;
        UpperShooterMotor.stopMotor();
    }

    @Override
    public boolean checkSystem() {
        return (
           true 
        );
    }
    @Override
    public void outputTelemetry(boolean isRobotDisabled) {
        SmartDashboard.putBoolean("Is Shooting", shooting);
        SmartDashboard.putNumber("Lower Shooter", LowerShooterMotor.get() * 100);
        SmartDashboard.putNumber("Upper Shooter", UpperShooterMotor.get() * 100);
    }
    
}