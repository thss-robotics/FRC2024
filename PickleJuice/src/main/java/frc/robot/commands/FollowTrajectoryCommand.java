package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;

public class FollowTrajectoryCommand extends Command {
    private final Drive drive = Drive.getInstance();
    private final Trajectory trajectory;
    private final RamseteController ramseteController = new RamseteController();
    private double startTime;

    public FollowTrajectoryCommand(Trajectory trajectory) {
        this.trajectory = trajectory;
    }

    @Override
    public void initialize() {
        drive.resetPose(trajectory.getInitialPose());
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double currentTime = Timer.getFPGATimestamp();
        double elapsedTime = currentTime - startTime;
        var desiredPose = trajectory.sample(elapsedTime);
        var refChassisSpeeds = ramseteController.calculate(drive.getPose(), desiredPose);
        drive.drive(refChassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= trajectory.getTotalTimeSeconds();
    }
}
