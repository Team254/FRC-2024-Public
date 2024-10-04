package com.team254.lib.util;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

import edu.wpi.first.wpilibj.DriverStation;

public class OSUtil {
    public static void fsSync() {
        try {
            System.out.println("Executing filesystem sync...");
            ProcessBuilder builder = new ProcessBuilder("sync");
            builder.redirectErrorStream(true);
            Process p = builder.start();
            BufferedReader r = new BufferedReader(new InputStreamReader(p.getInputStream()));
            String line;
            while (true) {
                line = r.readLine();
                if (line == null) {
                    break;
                }
            }
            System.out.println("Done");
        } catch (IOException e) {
            DriverStation.reportError("Failed to manually execute filesystem 'sync' command to flush logs to disk",
                    null);
        }
    }

    public static void fsSyncAsync() {
        new Thread(() -> {
            fsSync();
        }).start();
    }
}
