package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cscore.UsbCamera;

public class Superstructure {
    private static Superstructure instance;

    // States
    public enum RobotState {
        DRIVE,
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

    // Cameras
    private UsbCamera shooterCamera;
    private UsbCamera intakeCamera;
    private VideoSink server;

    // Ring Shoot Command
    private boolean isShootingRing = false;
    private Timer shootTimer = new Timer();
    private double shootWaitTime = Constants.kShootWaitTime; // Define this constant in your Constants class

    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }


    private Superstructure() {
        currentState = RobotState.DRIVE;
        previousState = RobotState.DRIVE;
        if(Constants.kEnableCameraServer){
            initCameras();
        }
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

   
    private void initCameras() {
        // Connect
        shooterCamera = CameraServer.startAutomaticCapture(0);
        intakeCamera = CameraServer.startAutomaticCapture(1);

        // Keep connected
        shooterCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        intakeCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

        // Init server
        server = CameraServer.getServer();
    }

    public void shootRing () {
        shooter.setShooterState(ShooterState.SHOOTING_AUTO, 1.0);
        isShootingRing = true;
        shootTimer.reset();
        shootTimer.start();
    }

    public void setState(RobotState newState) {
        previousState = currentState;
        currentState = newState;
        SmartDashboard.putString("Current State", currentState.toString());
        SmartDashboard.putString("Prev State", previousState.toString());
        switch (newState) {
            case DRIVE:
                arm.moveToDrivePosition();
                SmartDashboard.putBoolean("testing", previousState == RobotState.SPEAKER || previousState == RobotState.AMP);
                if(previousState == RobotState.SPEAKER || previousState == RobotState.AMP){
                    drive.setDriveMode(true);
                    server.setSource(intakeCamera);
                } else {
                    drive.setDriveMode(false);
                    server.setSource(shooterCamera);
                }
                
                break;
            case INTAKE:
                intake.setIntakeState(IntakeState.AUTO_INTAKING);
                arm.moveToIntakePosition();
                server.setSource(intakeCamera);
                drive.setIntakeMode();
                break;
            case SPEAKER:
                arm.moveToSpeakerPosition();
                server.setSource(shooterCamera);
                drive.setSpeakerMode();
                break;
            case AMP:
                arm.moveToAmpPosition();
                server.setSource(shooterCamera);
                drive.setAmpMode();
                break;
            case CHAIN:
                arm.moveToChainPosition();
                server.setSource(intakeCamera);
                drive.setChainMode();
                break;
        }
    }

    public RobotState getState() {
        return currentState;
    }

}
