package frc.robot;

import java.io.FileWriter;
import java.io.IOException;

public class Debug {

    public class DebugSettings {
        // Static variable to control whether logging is enabled
        public static boolean enableLogging = true;  // Change to false to disable logging
    }
    
    private FileWriter debugFileDriveSubsystem;

    public Debug(String filename) {
        if (DebugSettings.enableLogging) {
            try {
                this.debugFileDriveSubsystem = new FileWriter("media/sda1/" + filename);
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
    }

    public void logln(String logInfo) {
        this.log(logInfo + '\n');
    }

    public void log(String logInfo) {
        if (DebugSettings.enableLogging) {
            try {
                this.debugFileDriveSubsystem.write(logInfo);
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
    }

    public void closelog() {
        if (DebugSettings.enableLogging) {
            try {
                this.debugFileDriveSubsystem.close();
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
    }

}
