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

    AddressableLED ledStrip1;
    AddressableLED ledStrip2;
    AddressableLED ledStrip3;
    AddressableLED ledStrip4;

    AddressableLEDBuffer ledBuffer;
    int m_rainbowFirstPixelHue = 2;

    public static LED getInstance() {
        if (instance == null) {
            instance = new LED();
        }
        return instance;
    }

    private LED() {
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
