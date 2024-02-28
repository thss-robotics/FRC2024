package frc.robot.auto.actions;

import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.Timer;

public class EnableLimelightForDurationAction implements Action {

    private double mDuration;
    private double mInitialTimestamp;
    private Limelight mLimelight = Limelight.getInstance();

    public EnableLimelightForDurationAction(double duration) {
        mDuration = duration;
    }
    @Override
    public void start() {
        mInitialTimestamp = Timer.getFPGATimestamp();
        mLimelight.setDisableProcessing(false);
        mInitialTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mInitialTimestamp > mDuration;
    }

    @Override
    public void done() {
        mLimelight.setDisableProcessing(true);
    }
}
