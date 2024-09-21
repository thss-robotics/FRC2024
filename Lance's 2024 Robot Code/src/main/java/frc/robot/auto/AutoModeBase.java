package frc.robot.auto;

import frc.robot.auto.AutoManager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This
 * is implemented in auto modes (which are
 * routines that do actions).
 */
public abstract class AutoModeBase {
    protected double m_update_rate = 1.0 / 50.0;
    protected boolean m_active = false;
    protected boolean mIsInterrupted = false;


    protected double startTime = 0.0;

    protected double currentTime() {
        return Timer.getFPGATimestamp() - startTime;
    }

    protected abstract void routine() throws AutoModeEndedException;

    public void run() {
        m_active = true;
        try {
            routine();
        } catch (AutoModeEndedException e) {
            System.out.println("Auto mode done, ended early");
            return;
        }

        done();
        System.out.println("Auto mode done");
    }

    public void done() {
    }

    public void stop() {
        m_active = false;
    }

    public boolean isActive() {
        return m_active;
    }

    public void interrupt() {
        System.out.println("** Auto mode interrrupted!");
        mIsInterrupted = true;
    }

    public void resume() {
        System.out.println("** Auto mode resumed!");
        mIsInterrupted = false;
    }
    
    public boolean isActiveWithThrow() throws AutoModeEndedException {
        if (!isActive()) {
            throw new AutoModeEndedException();
        }

        return isActive();
    }

    public void runAction(AutoManager action) throws AutoModeEndedException {
        isActiveWithThrow();
        
        while (isActiveWithThrow() ) {
            action.getTrajectory();
            long waitTime = (long) (m_update_rate * 1000.0);

            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.done();
    }

    public abstract Pose2d getStartingPose();

    public boolean getIsInterrupted() {
        return mIsInterrupted;
    }
}