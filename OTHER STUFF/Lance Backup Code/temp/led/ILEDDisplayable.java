package frc.robot.led;

import com.ctre.phoenix.led.CANdle;

public interface ILEDDisplayable {
    void writePixels(CANdle candle);
}
