package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.Subsystem;

public class LED extends Subsystem {
    private static LED instance = null;

    private AddressableLED leds;
    private AddressableLEDBuffer ledBuffer;
    private LEDState currentState;
    private final int totalLength;

    private int rainbowFirstPixelHue = 2;

    // Define sections of the LED strip
    private enum Section {
        ARM, BODY, UNDER
    }

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
        leds = new AddressableLED(9); // Single PWM port for all LEDs
        totalLength = calculateTotalLength();
        ledBuffer = new AddressableLEDBuffer(totalLength);
        leds.setLength(ledBuffer.getLength());
        leds.setData(ledBuffer);
        leds.start();
        currentState = new OffState(); // Initial state
    }

    public void ledPeriodic() {
        currentState.execute();
        leds.setData(ledBuffer);
    }

    public void setState(LEDState newState) {
        if (currentState != null) {
            currentState.exit();
        }
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
                    LED.this.setState(new OffState()); // Switch back to off state after blinking
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


    private void setAllLEDsColor(int hue, int saturation, int value) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setHSV(i, hue, saturation, value);
        }
    }

    private int calculateTotalLength() {
        // Calculate the total length of the LED strip based on sections
        int armLength = 10;
        int bodyLength = 14 * 2;
        int underLength = 61;
        return armLength + bodyLength + underLength;
    }

    @Override
    public void stop() {
        // Implement method to stop LED animations or turn off LEDs
    }

    @Override
    public boolean checkSystem() {
        // Implement method to check LED system health, if necessary
        return true;
    }

    @Override
    public void outputTelemetry(boolean isRobotDisabled) {
        // Implement method to output LED-related telemetry, if necessary
    }
}
