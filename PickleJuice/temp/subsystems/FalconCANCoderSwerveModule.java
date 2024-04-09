package frc.robot.subsystems;

import com.ctre.phoenixpro.hardware.CANcoder;
import frc.lib.drivers.CanDeviceId;
import frc.lib.drivers.SwerveModule;
import frc.lib.geometry.Rotation2d;

// SDS Mk4i L3 Internals
public class FalconCANCoderSwerveModule extends SwerveModule {
    private final CANcoder mCanCoder;

    public FalconCANCoderSwerveModule(CanDeviceId driveId, CanDeviceId steeringId,  CANcoder cancoder, Rotation2d encoderZero) {
        super(driveId, steeringId, encoderZero);
        mCanCoder = cancoder;

        rezeroSteeringMotor();

        stop();
    }

    @Override
    public Rotation2d getSteerEncoderAngle() {
        return Rotation2d.fromDegrees(mCanCoder.getAbsolutePosition().refresh().getValue() * 360);
    }

}
