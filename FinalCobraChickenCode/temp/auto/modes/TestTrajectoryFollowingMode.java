package frc.robot.auto.modes;

import frc.robot.AutoModeSelector;
import frc.robot.auto.actions.*;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Superstructure;
import frc.lib.geometry.Rotation2d;

public class TestTrajectoryFollowingMode extends AutoModeBase {

    AutoModeSelector.DockMode mDockMode;
    public TestTrajectoryFollowingMode(AutoModeSelector.DockMode dockMode) {
        mDockMode = dockMode;
        addAction(new OrientModulesAction(1,Rotation2d.fromDegrees(0)));
        addAction(new SuperstructureAction(Superstructure.GoalState.UNSTOW));
        addAction(new SuperstructureAction(Superstructure.GoalState.STOW));
        for (int i = 0; i < 10; ++i) {
            addAction(new DriveTrajectoryAction(Drive.getInstance().getTrajectoryGenerator().getTrajectorySet().testTrajectory, false));
            addAction(new DriveTrajectoryAction(Drive.getInstance().getTrajectoryGenerator().getTrajectorySet().testTrajectory2, false));
        }

    }
}