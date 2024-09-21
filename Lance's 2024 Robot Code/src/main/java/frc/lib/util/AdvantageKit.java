package frc.lib.util;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import edu.wpi.first.networktables.*;

public class AdvantageKit {
    public static void startLog() {
        File[] mediaDevices = new File("/media/").listFiles();
        String logPath = "/home/lvuser/";
        for (File mediaDevice : mediaDevices) {
            if (mediaDevice.listFiles().length > 0) {
                logPath = mediaDevice.getAbsolutePath().concat("/Latest.wpilog");
                break;
            }
        }
        Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(new WPILOGWriter(logPath));
        SmartDashboard.putString("Log Path", logPath);
        Logger.start();
    }

    public static void endLog() {
        Logger.end();
    }

    public static void logAdvantageKit(NetworkTableEntry entry, String trackedTable, String key) {
        switch (entry.getType()) {
            case kDouble:
                Logger.recordOutput("Values/" + trackedTable + "/" + key, entry.getDouble(0.0));
                break;
            case kDoubleArray:
                Logger.recordOutput("Values/" + trackedTable + "/" + key.toString(),
                        entry.getDoubleArray(new double[] { 0.0, 0.0 }));
                break;
            case kString:
                Logger.recordOutput("Values/" + trackedTable + "/" + key.toString(), entry.getString(""));
                break;
            case kBoolean:
                Logger.recordOutput("Values/" + trackedTable + "/" + key.toString(), entry.getBoolean(false));
                break;
            default:
                Logger.recordOutput("Values/" + trackedTable + "/" + key.toString(),
                        "(AdvantageKit) Not tracking entry type " + entry.getType().toString());
                break;
        }
    }

    public static void recordMetadata() {
        Logger.recordMetadata("ProjectName", "FRC-TANK-DRIVE");
    }
}
