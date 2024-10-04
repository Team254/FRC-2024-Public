package com.team254.lib.auto;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;

/**
 * Tests to ensure that all choreo files have the same starting and ending
 * positions.
 */
public class ChoreoTest {
    private final String[] startPathIndicators = { "A-", "B-", "C-", "D-", "E-", "Truss-", "Speaker-", "Source-",
            "Amp-", "ACCW-", "BCCW-", "CCCW-", "DCCW-", "BCW-", "CCW-", "DCW-", "ECW-" };
    private HashMap<String, Pose2d> startPathChecker = new HashMap<>();

    private final String[] endPathIndicators = { "-A", "-B", "-C", "-D", "-E", "-ACCW", "-BCCW", "-CCCW", "-DCCW",
            "-BCW", "-CCW", "-DCW", "-ECW" };
    private HashMap<String, Pose2d> endPathChecker = new HashMap<>();

    @Test
    void testSameStartingPosition() throws IOException {
        for (File file : new File(Filesystem.getDeployDirectory(), "/choreo").listFiles()) {
            if (file.getName().split("\\.").length > 2)
                continue;
            for (String indicator : startPathIndicators) {
                if (file.getName().startsWith(indicator)) {
                    Pose2d startPose = AutoUtil.loadChoreoFile(file).getInitialPose();
                    if (!startPathChecker.keySet().contains(indicator)) {
                        startPathChecker.put(indicator, startPose);
                    }
                    if (!startPose.equals(startPathChecker.get(indicator))) {
                        Assertions.fail("Starting Pose Does Not Match! Filename: " + file.getName());
                    }
                }
            }
        }
    }

    @Test
    void testSameEndingPosition() throws IOException {
        for (File file : new File(Filesystem.getDeployDirectory(), "/choreo").listFiles()) {
            if (file.getName().split("\\.").length > 2)
                continue;
            // We explicitly allow end positions of some of our paths to diverge (for speed).
            if (file.getName().contains("NoPreload"))
                continue;
            if (file.getName().contains("A-Score-B"))
                continue;
            if (file.getName().contains("B-Score-A"))
                continue;
            for (String indicator : endPathIndicators) {
                if (file.getName().substring(0, file.getName().length() - 5).endsWith(indicator)) {
                    Pose2d endPose = AutoUtil.loadChoreoFile(file).getFinalPose();
                    if (!endPathChecker.keySet().contains(indicator)) {
                        endPathChecker.put(indicator, endPose);
                    }
                    if (!endPose.equals(endPathChecker.get(indicator))) {
                        Assertions.fail("Ending Pose Does Not Match! Filename: " + file.getName());
                    }
                }
            }
        }
    }
}