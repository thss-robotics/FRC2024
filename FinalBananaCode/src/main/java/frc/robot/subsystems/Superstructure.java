package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSink;
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
        shooterCamera = CameraServer.startAutomaticCapture(1);
        intakeCamera = CameraServer.startAutomaticCapture(0);
        server = CameraServer.getServer();
    }

    public void setState(RobotState newState) {
        currentState = newState;
        switch (newState) {
            case DRIVE:
                arm.moveToDrivePosition();
                server.setSource(shooterCamera);
                drive.setDriveMode();
                break;
            case INTAKE:
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
        // Place any code here that needs to run in every robot state
        led.ledPeriodic();
    }
}
