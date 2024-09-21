package frc.robot.auto;

import frc.robot.loops.CrashTrackingRunnable;

/**
 * This class selects, runs, and stops (if necessary) a specified autonomous
 * mode.
 */
public class AutoModeExecutor {
    private AutoModeBase m_auto_mode = null;
    private Thread m_thread = null;

    public void setAutoMode(AutoModeBase new_auto_mode) {
        m_auto_mode = new_auto_mode;
        m_thread = new Thread(new CrashTrackingRunnable() {
                @Override
                public void runCrashTracked() {
                    if (m_auto_mode != null) {
                        m_auto_mode.run();
                    }
                }
            });
    }

    public void start() {
        if (m_thread != null) {
            m_thread.start(); 
        }
    }
    
        public boolean isStarted() {
            return m_auto_mode != null && m_auto_mode.isActive() && m_thread != null && m_thread.isAlive();
        }
    
        public void reset() {
            if (isStarted()) {
                stop();
            }
    
            m_auto_mode = null;
        }
    

    public void stop() {
        if (m_auto_mode != null) {
            m_auto_mode.stop();
        }

        m_thread = null;
    }
    public AutoModeBase getAutoMode() {
        return m_auto_mode;
    }

    public boolean isInterrupted() {
        if (m_auto_mode == null) {
            return false;
        }
        return m_auto_mode.getIsInterrupted();
    }

    public void interrupt() {
        if (m_auto_mode == null) {
            return;
        }
        m_auto_mode.interrupt();
    }

    public void resume() {
        if (m_auto_mode == null) {
            return;
        }
        m_auto_mode.resume();
    }
}