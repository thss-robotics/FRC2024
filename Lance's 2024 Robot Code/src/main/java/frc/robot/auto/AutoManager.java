package frc.robot.auto;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.io.IOException;
import java.nio.file.Path;

import frc.robot.subsystems.Drive;

public abstract class AutoManager extends AutoModeBase {
    private static AutoManager instance = null;
    private Trajectory trajectory = new Trajectory();
    private String trajectoryJSON = "output/Test.wpilib.json";

    private AutoManager() {
        loadTrajectory();
    }

    private void loadTrajectory() {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

    protected abstract void routine() throws AutoModeEndedException;

    public abstract Pose2d getStartingPose();
}
