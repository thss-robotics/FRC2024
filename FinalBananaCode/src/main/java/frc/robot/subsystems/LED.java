package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.Subsystem;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Arm.ArmState;

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

        // Attach RGB strips
        ledStrip1 = new AddressableLED(0);
        ledStrip2 = new AddressableLED(1);
        ledStrip3 = new AddressableLED(2);
        ledStrip4 = new AddressableLED(3);

        ledBuffer = new AddressableLEDBuffer(60);

        ledStrip1.setLength(ledBuffer.getLength());
        ledStrip2.setLength(ledBuffer.getLength());
        ledStrip3.setLength(ledBuffer.getLength());
        ledStrip4.setLength(ledBuffer.getLength());


        // Set the data
        ledStrip1.setData(ledBuffer);
        ledStrip2.setData(ledBuffer);
        ledStrip3.setData(ledBuffer);
        ledStrip4.setData(ledBuffer);

        ledStrip1.start();
        ledStrip2.start();
        ledStrip3.start();
        ledStrip4.start();
    }

    public void ledPeriodic() {
        rainbow();

        ledStrip1.setData(ledBuffer);
        ledStrip2.setData(ledBuffer);
        ledStrip3.setData(ledBuffer);
        ledStrip4.setData(ledBuffer);
    }

    private void rainbow() {

    // For every pixel

    for (var i = 0; i < ledBuffer.getLength(); i++) {

        // Calculate the hue - hue is easier for rainbows because the color

        // shape is a circle so only one value needs to precess

        final var hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;

        // Set the value

        ledBuffer.setHSV(i, hue, 255, 128);

    }

    // Increase by to make the rainbow "move"

    m_rainbowFirstPixelHue += 3;

    // Check bounds

    m_rainbowFirstPixelHue %= 180;

    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'checkSystem'");
    }

    @Override
    public void outputTelemetry(boolean isRobotDisabled) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'outputTelemetry'");
    }
}
