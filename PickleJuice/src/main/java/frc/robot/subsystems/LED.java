package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Subsystem;
import frc.robot.subsystems.LED.DisabledState;

public class LED extends Subsystem {
    private static LED instance = null;

    private AddressableLED leds;
    private AddressableLEDBuffer ledBuffer;
    private LEDState currentState;
    private LEDState previousState;
    private final int armBodyLength = 14; // Arm and Body LEDs in parallel
    private final int underLength = 61; // Underlighting LEDs

    private int rainbowFirstPixelHue = 0;

    // State interface
    private interface LEDState {
        void enter();
        void execute();
        void exit();
    }

    public synchronized static LED getInstance() {
        if (instance == null) {
            instance = new LED();
        }
        return instance;
    }

    private LED() {
        // Connect strip
        leds = new AddressableLED(9);

        // Set up buffer
        int totalLength = armBodyLength + underLength;
        ledBuffer = new AddressableLEDBuffer(totalLength);
        leds.setLength(ledBuffer.getLength());
        leds.setData(ledBuffer);
        leds.start();

        currentState = new OffState();
        previousState = new OffState();
    }

    public void ledPeriodic() {
        currentState.execute();
        leds.setData(ledBuffer);
    }

    public void setState(LEDState newState) {
        if (currentState != null) {
            currentState.exit();
        }
        previousState = currentState;
        currentState = newState;
        if (currentState != null) {
            currentState.enter();
        }
    }

    // State implementations
    private class OffState implements LEDState {
        @Override
        public void enter() {
            setAllLEDsColor(0, 0, 0);
        }

        @Override
        public void execute() {
            // Nothing to do here
        }

        @Override
        public void exit() {
            // Cleanup if needed
        }
    }

    public class HealthNotifyState implements LEDState {
        private boolean systemsHealthy;
        private double lastBlinkTime;
        private int blinkCount;

        public HealthNotifyState(boolean systemsHealthy) {
            this.systemsHealthy = systemsHealthy;
        }

        @Override
        public void enter() {
            blinkCount = 0;
            lastBlinkTime = Timer.getFPGATimestamp();
        }

        @Override
        public void execute() {
            double currentTime = Timer.getFPGATimestamp();
            if (currentTime - lastBlinkTime >= 0.25) {
                if (blinkCount % 2 == 0) {
                    setAllLEDsColor(systemsHealthy ? 60 : 0, 255, 128); // Green or Red
                } else {
                    setAllLEDsColor(0, 0, 0); // Off
                }
                blinkCount++;
                lastBlinkTime = currentTime;
                if (blinkCount >= 4) {
                    LED.this.setState(previousState); // Switch back to off state after blinking
                }
            }
        }

        @Override
        public void exit() {
            setAllLEDsColor(0, 0, 0); // Ensure LEDs are turned off when exiting state
        }
    }

    // TeleopState: Greenish hues animation
    public class TeleopState implements LEDState {
        @Override
        public void enter() {}

        @Override
        public void execute() {
            // Simple greenish hues animation
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                int hue = 85 + (i % 20); // Hues around green
                ledBuffer.setHSV(i, hue, 255, 128);
            }
        }

        @Override
        public void exit() {}
    }

    // AutonomousState: Blue/Purple hue animation
    public class AutonomousState implements LEDState {
        @Override
        public void enter() {}

        @Override
        public void execute() {
            // Simple blue to purple animation
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                int hue = 180 + (i % 45); // Hues from blue to purple
                ledBuffer.setHSV(i, hue, 255, 128);
            }
        }

        @Override
        public void exit() {}
    }

    // DisabledState: Maintain the rainbow animation
    public class DisabledState implements LEDState {
        @Override
        public void enter() {}

        @Override
        public void execute() {
            // Rainbow effect
            for (var i = 0; i < ledBuffer.getLength(); i++) {
                final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
                ledBuffer.setHSV(i, hue, 255, 128);
            }
            rainbowFirstPixelHue += 3;
            rainbowFirstPixelHue %= 180;
        }

        @Override
        public void exit() {}
    }


    public class RingIntakeState implements LEDState {
        private double lastToggleTime;
        private boolean isPatternVisible = false;

        @Override
        public void enter() {
            lastToggleTime = Timer.getFPGATimestamp();
        }

        @Override
        public void execute() {
            double currentTime = Timer.getFPGATimestamp();
            if (currentTime - lastToggleTime >= 0.5) { // Toggle every 0.5 seconds
                isPatternVisible = !isPatternVisible;
                if (isPatternVisible) {
                    applyPattern();
                } else {
                    setAllLEDsColor(0, 0, 0); // Turn off all LEDs
                }
                lastToggleTime = currentTime;
            }
        }

        @Override
        public void exit() {
            setAllLEDsColor(0, 0, 0); // Ensure LEDs are turned off when exiting state
        }

        private void applyPattern() {
            // Apply a pattern here, potentially different for arm/body and underlighting
            for (int i = 0; i < armBodyLength; i++) {
                ledBuffer.setHSV(i, 100, 255, 128); // Example hue for arm/body
            }
            for (int i = armBodyLength; i < ledBuffer.getLength(); i++) {
                ledBuffer.setHSV(i, 180, 255, 128); // Different hue for underlighting
            }
        }
    }

    private void setAllLEDsColor(int hue, int saturation, int value) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setHSV(i, hue, saturation, value);
        }
    }

    @Override
    public void stop() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
        leds.setData(ledBuffer);
        leds.stop();
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry(boolean isRobotDisabled) {
        SmartDashboard.putString("LED Current State", currentState.toString());
    }
}