package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeState;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cscore.UsbCamera;

public class Superstructure {
    private static Superstructure instance;

    public enum RobotState {
        DRIVE,
        INTAKE,
        SPEAKER,
        AMP,
        CHAIN
    }

    private RobotState currentState;
    private RobotState previousState;

    private final Shooter shooter = Shooter.getInstance();
    private final Drive drive = Drive.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Arm arm = Arm.getInstance();
    private final LED led = LED.getInstance();
    private UsbCamera shooterCamera;
    private UsbCamera intakeCamera;
    private VideoSink server;

    private Superstructure() {
        currentState = RobotState.DRIVE;
        previousState = RobotState.DRIVE;
        if(Constants.kEnableCameraServer){
            initCameras();
        }
    }

    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    private void initCameras() {
        shooterCamera = CameraServer.startAutomaticCapture(0);
        intakeCamera = CameraServer.startAutomaticCapture(1);
        server = CameraServer.getServer();
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

    public void execute() {
        led.ledPeriodic();
        intake.intakePeriodic();
        drive.updateOdometry();
    }
}
