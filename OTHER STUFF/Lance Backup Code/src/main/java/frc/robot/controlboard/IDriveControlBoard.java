package frc.robot.controlboard;

public interface IDriveControlBoard {
    double getThrottle();

    double getStrafe();

    double getRotation();

    boolean resetGyro();

    boolean getSwitchDriveMode();

    boolean getScore();

    boolean getAutoAlign();
}