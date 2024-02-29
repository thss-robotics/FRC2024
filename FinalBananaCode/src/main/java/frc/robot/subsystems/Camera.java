package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import frc.lib.Subsystem;
import frc.robot.Constants;

public class Camera extends Subsystem{
    private static Camera instance = null;

    // Cameras
    private UsbCamera shooterCamera;
    private UsbCamera intakeCamera;
    private VideoSink server;

    // States
    public enum CameraState {
        DISABLED,
        INTAKE,
        SPEAKER,
        AMP,
        DRIVE,
    }

    private CameraState currentState = CameraState.DRIVE;

    public static Camera getInstance() {
        if (instance == null) {
            instance = new Camera();
        }
        return instance;
    }

    private Camera(){
        if(Constants.kEnableCameraServer){
            // Connect
            shooterCamera = CameraServer.startAutomaticCapture(0);
            intakeCamera = CameraServer.startAutomaticCapture(1);

            // Keep connected
            shooterCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
            intakeCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

            // Init server
            server = CameraServer.getServer();
        }
    }

    public void setCameraDriveMode(){

    }

    public void setCameraDriveReverseMode(){

    }

    public void setCameraIntakeMode(){

    }

    public void setCameraSpeakerMode(){

    }

    public void setCameraAmpMode(){

    }

    public void setCameraChainMode(){

    }

    @Override
    public void stop() {
        
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry(boolean isRobotDisabled) {
       
    }
    
}
