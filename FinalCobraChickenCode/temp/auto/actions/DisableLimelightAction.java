package frc.robot.auto.actions;

import frc.robot.subsystems.Limelight;

public class DisableLimelightAction implements Action {

    private Limelight mLimelight = Limelight.getInstance();

    public DisableLimelightAction() {
    }

    @Override
    public void start() {
        mLimelight.setDisableProcessing(true);
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {
    }
}
