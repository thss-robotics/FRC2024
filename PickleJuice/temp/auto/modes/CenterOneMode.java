package frc.robot.auto.modes;

import frc.robot.AutoModeSelector;
import frc.robot.auto.actions.*;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Superstructure;
import frc.lib.geometry.Rotation2d;

/**
 * Center starting location - Scores one high cone. Ends with bridge dock.
 */
public class CenterOneMode extends AutoModeBase {

    AutoModeSelector.DockMode mDockMode;
    public CenterOneMode(AutoModeSelector.DockMode dockMode) {
        mDockMode = dockMode;
        addAction(new ParallelAction(
                new OrientModulesAction(0.8, Rotation2d.fromDegrees(0)),
                new SeriesAction(
                        new SuperstructureAction(Superstructure.GoalState.HIGH_SCORING_CONE),
                        new SuperstructureAction(Superstructure.GoalState.SCORE)
                )
        ));

        if (mDockMode == AutoModeSelector.DockMode.DOCK) {
            DockActionNew dockAction = new DockActionNew(DockActionNew.StartingSide.NEAR, DockActionNew.ApproachOrientation.BACK_FIRST, 1.0, 15);
            addAction(new ParallelAction(
                    new DisableLimelightAction(),
                    new SuperstructureAction(Superstructure.GoalState.STOW),
                    new InterruptableAction(new DriveTrajectoryAction(Drive.getInstance().getTrajectoryGenerator().getTrajectorySet().coopFirstScoreToDock), dockAction::getInclinationAboveThresholdForDock))
            );

            addAction(dockAction);
        } else {
            addAction(
                    new SuperstructureAction(Superstructure.GoalState.STOW)
            );
        }
    }
}
