package frc.robot.subsystems;

import frc.lib.Subsystem;
import frc.robot.Constants;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Superstructure extends Subsystem{
    private static Superstructure instance;

    // States
    public enum RobotState {
        DRIVE,
        DRIVE_REVERSE,
        INTAKE,
        SPEAKER,
        AMP,
        CHAIN
    }
    private RobotState currentState;
    private RobotState previousState;

    // Subsystems
    private final Shooter shooter = Shooter.getInstance();
    private final Drive drive = Drive.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Arm arm = Arm.getInstance();
    private final LED led = LED.getInstance();
    private final Camera camera = Camera.getInstance();

    private final SubsystemManager subsystemManager = SubsystemManager.getInstance();

    // Ring Shoot Command
    private boolean isShootingRing = false;
    private Timer shootTimer = new Timer();
    private double shootWaitTime = Constants.kShootWaitTime;

    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    private Superstructure() {
        currentState = RobotState.DRIVE;
        previousState = RobotState.DRIVE;
    }

     public void superstructurePeriodic() {
        led.ledPeriodic();
        intake.intakePeriodic();
        drive.updateOdometry();

        // Handle ring shooting
        if (isShootingRing) {
            if (shootTimer.hasElapsed(shootWaitTime)) {
                intake.setIntakeState(Intake.IntakeState.SHOOTING);
                if (!intake.ringLoaded()) {
                    setState(RobotState.DRIVE);
                    shooter.setShooterState(Shooter.ShooterState.OFF, 0);
                    intake.setIntakeState(Intake.IntakeState.OFF);
                    isShootingRing = false;
                    shootTimer.stop();
                }
            }
        }
    }

    public void setState(RobotState newState) {
        previousState = currentState;
        currentState = newState;

        switch (newState) {
            case DRIVE:
                if(previousState == RobotState.SPEAKER || previousState == RobotState.AMP){
                    setState(RobotState.DRIVE_REVERSE);
                    break;
                } else {
                    drive.setDriveMode(false);
                }
                arm.moveToDrivePosition();
                camera.setCameraDriveMode();
                break;
            case DRIVE_REVERSE:
                arm.moveToDriveReversePosition();
                drive.setDriveMode(true);
                camera.setCameraDriveReverseMode();
                break;
            case INTAKE:
                intake.setIntakeState(IntakeState.AUTO_INTAKING);
                arm.moveToIntakePosition();
                drive.setIntakeMode();
                camera.setCameraIntakeMode();
                break;
            case SPEAKER:
                arm.moveToSpeakerPosition();
                drive.setSpeakerMode();
                camera.setCameraSpeakerMode();
                break;
            case AMP:
                arm.moveToAmpPosition();
                drive.setAmpMode();
                camera.setCameraAmpMode();
                break;
            case CHAIN:
                arm.moveToChainPosition();
                drive.setChainMode();
                camera.setCameraChainMode();
                break;
        }
    }

    public RobotState getState() {
        return currentState;
    }

    public void shootRing () {
        shooter.setShooterState(ShooterState.SHOOTING_AUTO, 1.0);
        isShootingRing = true;
        shootTimer.reset();
        shootTimer.start();
    }

    @Override
    public void stop() {
        subsystemManager.stop();
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public synchronized void outputTelemetry(boolean isRobotDisabled) {
        SmartDashboard.putBoolean("Shooting Ring", isShootingRing);
    }
}
