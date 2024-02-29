package frc.robot.auto.actions;

import frc.robot.RobotState;
import frc.robot.planners.AutoAlignPointSelector;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Superstructure;
import frc.lib.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;

public class ScoreAction implements Action {
    protected final Drive mDrive = Drive.getInstance();
    protected final Superstructure mSuperstructure = Superstructure.getInstance();
    protected final Superstructure.GoalState mDesiredState;

    private boolean pieceScored = false;

    public ScoreAction(Superstructure.GoalState goal) {
        mDesiredState = goal;
    }

    @Override
    public void start() {
        double t = Timer.getFPGATimestamp();
        Optional<Pose2d> targetPoint = AutoAlignPointSelector.chooseTargetPoint(RobotState.getInstance().getFieldToVehicle(t),
                mSuperstructure.getGameObjectType() == Superstructure.GameObjectType.CONE ? AutoAlignPointSelector.RequestedAlignment.AUTO_CONE : AutoAlignPointSelector.RequestedAlignment.AUTO_CUBE);
        targetPoint.ifPresent(mDrive::setSnapToPoint);
        mSuperstructure.setGoalState(mDesiredState);
    }

    @Override
    public void update() {
        Superstructure.GoalState newGoal = Superstructure.GoalState.SCORE;
        if(mDrive.getAutoAlignComplete() && mSuperstructure.isAtGoal()) {
            mSuperstructure.setGoalState(newGoal);
        }
        if(mSuperstructure.getState().isInRange(newGoal.state)) {
            pieceScored = true;
        }
    }

    @Override
    public boolean isFinished() {
        return pieceScored;
    }

    @Override
    public void done() {}
}