package frc.robot;

import frc.robot.auto.modes.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {
    enum DesiredMode {
        DO_NOTHING,
        TEST_TRAJECTORY,
    }

    private DesiredMode cachedDesiredMode = DesiredMode.DO_NOTHING;

    private final SendableChooser<DesiredMode> modeChooser;


    public AutoModeSelector() {
        modeChooser = new SendableChooser<>();
        modeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        modeChooser.addOption("Test Trajectory", DesiredMode.TEST_TRAJECTORY);
        SmartDashboard.putData("Auto mode", modeChooser);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = modeChooser.getSelected();

        if (desiredMode == null) {
            desiredMode = DesiredMode.DO_NOTHING;
        }

        if (cachedDesiredMode != desiredMode) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name());
        }
        cachedDesiredMode = desiredMode;
    }

    private void getAutoModeForParams(DesiredMode mode) {
        switch (mode) {
            case DO_NOTHING:
                break;
            case TEST_TRAJECTORY:
                break;
            default:
                break;
        }
        return;
    }

    public void reset() {
        cachedDesiredMode = DesiredMode.DO_NOTHING;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", cachedDesiredMode.name());
    }
}