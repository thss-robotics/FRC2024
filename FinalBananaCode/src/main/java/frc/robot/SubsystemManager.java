package frc.robot;

import frc.lib.Subsystem;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.loops.Looper;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */

public class SubsystemManager implements ILooper {
    public static SubsystemManager mInstance = null;

    private List<Subsystem> allSubsystems = new ArrayList<>();
    private List<Loop> loops = new ArrayList<>();

    private SubsystemManager() {}

    public static SubsystemManager getInstance() {
        if (mInstance == null) {
            mInstance = new SubsystemManager();
        }

        return mInstance;
    }

    public void outputToSmartDashboard(boolean disabled) {
        allSubsystems.forEach(s -> s.outputTelemetry(disabled));
    }

    public boolean checkSubsystems() {
        boolean ret_val = true;

        for (Subsystem s : allSubsystems) {
            ret_val &= s.checkSystem();
        }

        return ret_val;
    }

    public boolean checkDeviceConfigurations() {
        boolean ret_val = true;

        for (Subsystem s : allSubsystems) {
            boolean areDevicesOK = s.checkDeviceConfiguration();
            if (!areDevicesOK) {
                s.rewriteDeviceConfiguration();
                DriverStation.reportWarning("Rewriting configuration after detecting configuration drift for subsystem: " + s.toString(), false);
            }
            ret_val &= areDevicesOK;
        }

        return ret_val;
    }

    public void stop() {
        allSubsystems.forEach(Subsystem::stop);
    }

    public List<Subsystem> getSubsystems() {
        return allSubsystems;
    }

    public void setSubsystems(Subsystem... allSubsystems) {
        this.allSubsystems = Arrays.asList(allSubsystems);
    }

    private class EnabledLoop implements Loop {
        @Override
        public void onStart(double timestamp) {
            loops.forEach(l -> l.onStart(timestamp));
        }

        @Override
        public void onLoop(double timestamp) {
            allSubsystems.forEach(Subsystem::readPeriodicInputs);
            loops.forEach(l -> l.onLoop(timestamp));
            allSubsystems.forEach(Subsystem::writePeriodicOutputs);
        }

        @Override
        public void onStop(double timestamp) {
            loops.forEach(l -> l.onStop(timestamp));
        }
    }

    private class DisabledLoop implements Loop {
        private double mLastCheckTimeSeconds = 0;

        @Override
        public void onStart(double timestamp) {}

        @Override
        public void onLoop(double timestamp) {
            allSubsystems.forEach(Subsystem::readPeriodicInputs);

            // periodically check that device configurations are OK
            if ((Timer.getFPGATimestamp() - mLastCheckTimeSeconds) > Constants.kRecheckDeviceIntervalSeconds) {
                mLastCheckTimeSeconds = Timer.getFPGATimestamp();
                checkDeviceConfigurations();
            }
        }

        @Override
        public void onStop(double timestamp) {}
    }

    public void registerEnabledLoops(Looper enabledLooper) {
        allSubsystems.forEach(s -> s.registerEnabledLoops(this));
        enabledLooper.register(new EnabledLoop());
    }

    public void registerDisabledLoops(Looper disabledLooper) {
        disabledLooper.register(new DisabledLoop());
    }

    @Override
    public void register(Loop loop) {
        loops.add(loop);
    }
}
