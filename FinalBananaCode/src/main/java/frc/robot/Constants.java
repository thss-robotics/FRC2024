package frc.robot;

public class Constants {
    public static final double kLooperDt = 0.01;
    public static final double kEpsilon = 1E-3;

    // CAN
    public static int kFrontLeftDriveId = 1;
    public static int kFrontRightDriveId = 2;
    public static int kBackLeftDriveId = 3;
    public static int kBackRightDriveId = 4;

    // Controls
    public static final boolean kForceDriveGamepad = false;
    public static final int kDriveGamepadPort = 0;
    public static final int kMainLeftJoystickPort = 1;
    public static final int kMainRightJoystickPort = 2;
    public static final int kOperatorControllerPort = 3;
    public static final double kDriveJoystickThreshold = 0.075;
    public static final double kJoystickThreshold = 0.1;

    // Devices
    public static Boolean kEnableCameraServer = true;

    // Config
    public static Double kRecheckDeviceIntervalSeconds = 10.0;
    
    // Arm Positions
    public static final double kArmDriveAbsolutePosition = 0.156;
    public static final double kArmIntakeAbsolutePosition = 0.003;
    public static final double kArmSpeakerAbsolutePosition = 0.0582;
    public static final double kArmAmpAbsolutePosition = 0.262;
    public static final double kArmChainAbsolutePosition = 0.247;
}
