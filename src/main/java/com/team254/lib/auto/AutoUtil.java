package com.team254.lib.auto;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team254.lib.ctre.swerve.SwerveRequest;
import com.team254.lib.pathplanner.AdvancedAutoBuilder;

import org.littletonrobotics.junction.Logger;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.google.gson.Gson;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team254.frc2024.Constants;
import com.team254.frc2024.RobotContainer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class AutoUtil {
    public enum MidlineNote {
        A(0),
        B(1),
        C(2),
        D(3),
        E(4);

        private final int index;

        private MidlineNote(int index) {
            this.index = index;
        }

        public int getIndex() {
            return index;
        }

        public static MidlineNote fromIndex(int index) {
            switch (index) {
                case 0:
                    return MidlineNote.A;
                case 1:
                    return MidlineNote.B;
                case 2:
                    return MidlineNote.C;
                case 3:
                    return MidlineNote.D;
                case 4:
                    return MidlineNote.E;
                default:
                    return MidlineNote.A;
            }
        }
    }

    public enum MidlineNoteRobotOrientation {
        Straight(0),
        CW(1),
        CCW(2);

        private final int index;

        private MidlineNoteRobotOrientation(int index) {
            this.index = index;
        }

        public int getIndex() {
            return index;
        }

        public static MidlineNoteRobotOrientation fromIndex(int index) {
            switch (index) {
                case 0:
                    return MidlineNoteRobotOrientation.Straight;
                case 1:
                    return MidlineNoteRobotOrientation.CW;
                case 2:
                    return MidlineNoteRobotOrientation.CCW;
                default:
                    return MidlineNoteRobotOrientation.Straight;
            }
        }
    }

    public enum PriorityMidlineSequence {
        ABC(MidlineNote.A),
        ACB(MidlineNote.A),
        BAC(MidlineNote.B),
        BCA(MidlineNote.B),
        CAB(MidlineNote.C),
        CBA(MidlineNote.C),
        EDC(MidlineNote.E),
        DEC(MidlineNote.D);

        private final MidlineNote firstNote;

        private PriorityMidlineSequence(MidlineNote firstNote) {
            this.firstNote = firstNote;
        }

        public MidlineNote getFirstNote() {
            return firstNote;
        }
    }

    public enum StartingLocation {
        Truss,
        Speaker,
        Source,
        SpeakerCorner,
        Amp
    }

    public enum FirstAction {
        ThreeCloseToMidline("3"),
        OneCloseToMidline("1"),
        SprintToMidline("Sprint"),
        SprintToMidlineNoPreload("NoPreloadSprint");

        private final String choreoActionName;

        private FirstAction(String name) {
            this.choreoActionName = name;
        }

        public String getName() {
            return choreoActionName;
        }
    }

    public enum LastAction {
        ScorePreload,
        ScorePreloadAndThreeClose,
        ThreeClose,
        Backoff
    }

    private static final Gson gson = new Gson();

    public static ChoreoTrajectory loadChoreoFile(File path) {
        try {
            var reader = new BufferedReader(new FileReader(path));
            ChoreoTrajectory traj = gson.fromJson(reader, ChoreoTrajectory.class);

            return traj;
        } catch (Exception ex) {
            DriverStation.reportError(ex.getMessage(), ex.getStackTrace());
        }
        return null;
    }

    public static ChoreoTrajectory getChoreoTrajectory(boolean isRedAlliance, String pathName) {
        var trajectory = Choreo.getTrajectory(pathName);
        if (trajectory == null) {
            DriverStation.reportError("NULL CHOREO TRAJECTORY", true);
            return new ChoreoTrajectory();
        }
        return isRedAlliance ? trajectory.flipped() : trajectory;
    }

    public static ArrayList<ChoreoTrajectory> getChoreoTrajectories(boolean isRedAlliance, String pathName) {
        ArrayList<ChoreoTrajectory> trajectories = Choreo.getTrajectoryGroup(pathName);
        if (isRedAlliance) {
            for (int trajNum = 0; trajNum < trajectories.size(); trajNum++) {
                trajectories.set(trajNum, trajectories.get(trajNum).flipped());
            }
        }
        return trajectories;
    }

    private static ArrayList<String> getExistingChoreoPathSplitNames(String basePathName) {
        // Count files matching the pattern for split parts.
        var traj_dir = new File(Filesystem.getDeployDirectory(), "choreo");
        File[] files = traj_dir.listFiles((file) -> file.getName().matches(basePathName + "\\.\\d+\\.traj"));
        int segmentCount = files.length;
        ArrayList<String> fileNames = new ArrayList<>();
        for (int i = 1; i <= segmentCount; ++i) {
            String name = String.format("%s.%d.traj", basePathName, i);
            fileNames.add(name.substring(0, name.length() - 5));
        }
        return fileNames;
    }

    public static ArrayList<Command> getPathPlannerSwerveCommandsFromChoreoSplitNames(ArrayList<String> pathNames) {
        ArrayList<Command> commands = new ArrayList<>();
        for (String pathName : pathNames) {
            PathPlannerPath choreoPath = PathPlannerPath.fromChoreoTrajectory(pathName);

            commands.add(AdvancedAutoBuilder.followPath(choreoPath));
        }
        return commands;
    }

    public static Command getPathPlannerSwerveCommandFromChoreoName(String pathName) {
        PathPlannerPath choreoPath = getPathPlannerPathFromChoreoPathName(pathName);
        return AdvancedAutoBuilder.followPath(choreoPath);
    }

    public static List<Command> getPathPlannerSwerveCommandsFromChoreoPathName(String basePathName,
            boolean useSplitTrajectories) {
        return getPathPlannerSwerveCommandsFromPathPlannerPaths(
                getPathPlannerPathsFromChoreoPathName(basePathName, useSplitTrajectories));
    }

    public static List<PathPlannerPath> getPathPlannerPathsFromChoreoPathName(String basePathName,
            boolean useSplitTrajectories) {
        ArrayList<PathPlannerPath> paths = new ArrayList<>();
        if (!useSplitTrajectories) {
            var path = getPathPlannerPathFromChoreoPathName(basePathName);
            if (path == null) {
                return null;
            }
            return List.of(path);
        }
        ArrayList<String> pathNames = getExistingChoreoPathSplitNames(basePathName);
        // if path split size is 0, fall back to base path name
        if (pathNames.size() == 0) {
            var path = getPathPlannerPathFromChoreoPathName(basePathName);
            if (path == null) {
                return null;
            }
            return List.of(path);
        }
        // use split path trajectories
        else {
            for (String pathName : pathNames) {
                var path = getPathPlannerPathFromChoreoPathName(pathName);
                if (path == null) {
                    return null;
                }
                paths.add(path);
            }
        }
        return paths;
    }

    public static List<Command> getPathPlannerSwerveCommandsFromPathPlannerPaths(
            List<PathPlannerPath> pathPlannerPaths) {
        if (pathPlannerPaths == null)
            return Arrays.asList(Commands.none());
        ArrayList<Command> commands = new ArrayList<>();
        for (int i = 0; i < pathPlannerPaths.size(); i++) {
            commands.add(AdvancedAutoBuilder.followPath(pathPlannerPaths.get(i)));
        }
        return commands;
    }

    public static Command getPathPlannerSwerveCommandFromPathPlannerPath(PathPlannerPath pathPlannerPath) {
        if (pathPlannerPath == null)
            return Commands.none();
        return AdvancedAutoBuilder.followPath(pathPlannerPath);
    }

    public static Command getPathPlannerSwerveCommandWithNoteUpdateFromPathPlannerPath(PathPlannerPath pathPlannerPath,
            Command updateNote) {
        if (pathPlannerPath == null)
            return Commands.none();
        return AdvancedAutoBuilder.followPath(pathPlannerPath).alongWith(updateNote);
    }

    /**
     * Uses Choreo path name to generate a Path Planner
     * swerve command.
     */
    @SuppressWarnings("unused")
    private static Command getPathPlannerSwerveCommandFromChoreoPathName(String pathName) {

        return AdvancedAutoBuilder.followPath(getPathPlannerPathFromChoreoPathName(pathName));
    }

    public static PathPlannerPath getPathPlannerPathFromChoreoPathName(String pathName) {
        try {
            PathPlannerPath choreoPath = PathPlannerPath.fromChoreoTrajectory(pathName);
            return choreoPath;
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Uses one of the preloaded Choreo trajectories to generate a swerve command.
     * Uses Choreo library.
     */
    public static Command getChoreoSwerveCommand(RobotContainer container, String path) {
        ChoreoTrajectory traj = Choreo.getTrajectory(path);

        return getChoreoSwerveCommandFromTrajectory(container, traj);
    }

    public static Command getChoreoSwerveCommandFromTrajectory(RobotContainer container, ChoreoTrajectory traj) {
        if (traj == null)
            return Commands.none();

        Command swerveCommand = Commands.none();
        Command logPose = Commands.none();

        PIDController thetaController = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        swerveCommand = Choreo.choreoSwerveCommand(
                traj,
                () -> (container.getRobotState().getLatestFieldToRobot().getValue()),
                new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0),
                new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0),
                thetaController,
                (ChassisSpeeds speeds) -> container.getDriveSubsystem()
                        .setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)),
                () -> false,
                container.getDriveSubsystem());
        logPose = Commands.runOnce(
                () -> Logger.recordOutput("autofollower/Auto Pose",
                        container.getRobotState().getLatestFieldToRobot().getValue()));

        return new ParallelCommandGroup(swerveCommand, logPose);
    }

    public static ArrayList<Command> getChoreoSwerveCommandsFromTrajectories(RobotContainer container,
            ArrayList<ChoreoTrajectory> trajectories) {
        if (trajectories == null)
            return new ArrayList<>();
        ArrayList<Command> commands = new ArrayList<>();
        for (ChoreoTrajectory traj : trajectories) {
            commands.add(getChoreoSwerveCommandFromTrajectory(container, traj));
        }
        return commands;
    }

    /**
     * Flips X across field length
     */
    public static Translation2d flipTranslation(Translation2d translation) {
        return new Translation2d(Constants.kFieldLengthMeters - translation.getX(), translation.getY());
    }

    /**
     * This method checks if the current drive translation is within a rectangular
     * region.
     * Field Frame: x = 0 at blue driver station wall, y = 0 on feed station side
     *
     * @returns if current drive translation is within rectangular region
     * @param bottomLeft  bottom left corner of rectangle, lower x, higher y
     * @param topRight    top right corner of rectangle, higher x, lower y
     * @param useAlliance if true, will change bottomLeft and topRight to match
     *                    field frame based on alliance
     */
    public static boolean withinRegion(RobotContainer container, Translation2d bottomLeft, Translation2d topRight,
            boolean useAlliance) {
        Pose2d drivePose = container.getRobotState().getLatestFieldToRobot().getValue();
        if (useAlliance && container.getRobotState().isRedAlliance()) {
            drivePose = new Pose2d(flipTranslation(drivePose.getTranslation()), drivePose.getRotation());
        }
        return drivePose.getTranslation().getX() < topRight.getX()
                && drivePose.getTranslation().getX() > bottomLeft.getX()
                && drivePose.getTranslation().getY() < bottomLeft.getY()
                && drivePose.getTranslation().getY() > topRight.getY();
    }
}
