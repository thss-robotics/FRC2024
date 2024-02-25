package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import frc.lib.Subsystem;

public class LED extends Subsystem {
    private static LED instance = null;

    public enum WantedAction {
        DISPLAY_BATTERY_LOW,
        DISPLAY_GOOD_BATTERY,
        DISPLAY_NOT_HOMED,
        DISPLAY_SUPERSTRUCTURE,
        DISPLAY_VISION,
        DISPLAY_CONFIGURE_FAULT,
        OFF
    }

    private enum SystemState {
        DISPLAYING_BATTERY_LOW,
        DISPLAYING_CONFIGURE_FAULT,
        DISPLAYING_GOOD_BATTERY,
        DISPLAYING_NOT_HOMED,
        DISPLAYING_SUPERSTRUCTURE,
        DISPLAYING_VISION,
        OFF
    }

    AddressableLED led = new AddressableLED(9);

    private SystemState systemState = SystemState.OFF;
    private WantedAction wantedAction = WantedAction.OFF;

    public synchronized static LED getInstance() {
        if (instance == null) {
            instance = new LED();
        }
        return instance;
    }

    private LED() {
        led = new AddressableLED(9);
    }
  
    @Override
    public boolean checkSystem() { return true; }

    @Override
    public synchronized void outputTelemetry(boolean disabled) {}

    @Override
    public void stop() {}

}
