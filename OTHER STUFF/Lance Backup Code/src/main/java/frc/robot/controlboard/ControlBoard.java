package  frc.robot.controlboard;

import  frc.robot.Constants;

import edu.wpi.first.wpilibj.DriverStation;

public class ControlBoard implements IDriveControlBoard, IButtonControlBoard {
    private static ControlBoard mInstance = null;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }

        return mInstance;
    }

    private final IDriveControlBoard driveControlBoard;
    private final IButtonControlBoard buttonControlBoard;

    private ControlBoard() {
        boolean useDriveGamepad = Constants.kForceDriveGamepad ||
                DriverStation.getJoystickIsXbox(Constants.kDriveGamepadPort);
        driveControlBoard = useDriveGamepad ? GamepadDriveControlBoard.getInstance()
                : MainDriveControlBoard.getInstance();
        buttonControlBoard = GamepadButtonControlBoard.getInstance();
    }

    @Override
    public double getThrottle() {
        return driveControlBoard.getThrottle();
    }

    @Override
    public double getStrafe() {
        return driveControlBoard.getStrafe();
    }

    @Override
    public double getRotation() {
        return driveControlBoard.getRotation();
    }

    @Override
    public boolean resetGyro() {
        return driveControlBoard.resetGyro();
    }

    @Override
    public boolean getTestButton() {
        return buttonControlBoard.getTestButton();
    }

    @Override
    public boolean getSwitchDriveMode() {
        return driveControlBoard.getSwitchDriveMode();
    }

    @Override
    public boolean getScoreHigh() {
        return buttonControlBoard.getScoreHigh();
    }

    @Override
    public boolean getScoreMedium() {
        return buttonControlBoard.getScoreMedium();
    }

    @Override
    public boolean getScoreLow() {
        return buttonControlBoard.getScoreLow();
    }

    @Override
    public boolean primeLowFling() {
        return buttonControlBoard.primeLowFling();
    }

    @Override
    public boolean primeHighFling() {
        return buttonControlBoard.primeHighFling();
    }

    @Override
    public boolean getScoreMid() {
        return buttonControlBoard.getScoreMid();
    }

    @Override
    public boolean getPickupFeederCone() {
        return buttonControlBoard.getPickupFeederCone();
    }

    @Override
    public boolean getPickupGroundCone() {
        return buttonControlBoard.getPickupGroundCone();
    }

    @Override
    public boolean getPickupFeederCube() {
        return buttonControlBoard.getPickupFeederCube();
    }

    @Override
    public boolean getPickupGroundCube() {
        return buttonControlBoard.getPickupGroundCube();
    }

    @Override
    public boolean getScore() {
        return driveControlBoard.getScore();
    }

    @Override
    public boolean getStow() {
        return buttonControlBoard.getStow();
    }

    @Override
    public boolean getOpenClaw() {
        return buttonControlBoard.getOpenClaw();
    }

    @Override
    public boolean getCloseClaw() {
        return buttonControlBoard.getCloseClaw();
    }

    @Override
    public boolean getAutoAlign() {
        return driveControlBoard.getAutoAlign();
    }

    @Override
    public boolean getToggleClimbMode() {
        return buttonControlBoard.getToggleClimbMode();
    }

    @Override
    public double getManualForksJog() {
        return buttonControlBoard.getManualForksJog();
    }

    @Override
    public boolean getForksUp() {
        return buttonControlBoard.getForksUp();
    }

    @Override
    public boolean getForksDown() {
        return buttonControlBoard.getForksDown();
    }

    @Override
    public boolean getFling() {
        return buttonControlBoard.getFling();
    }

    @Override
    public boolean getGroundIntakePush() { return buttonControlBoard.getGroundIntakePush(); }
}