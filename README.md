# FRC-2024-Public
Team 254's 2024 FRC robot code for [Vortex](https://www.team254.com/first/2024/). Vortex's code is written in Java and is based off of WPILib's Java control system.

The code is divided into several packages, each responsible for a different aspect of the robot function. This README explains setup instructions, the function of each package, and some of the variable naming conventions used. Additional information about each specific class can be found in that class' Java file.

## Setup Instructions

### General
1. Clone this repo
1. Run `./gradlew` to download gradle and needed FRC/Vendor libraries
1. Run `./gradlew tasks` to see available options
1. Enjoy!

### Visual Studio Code (Official IDE)
1. Get the WPILib extension for easiest use from the VSCode Marketplace - Requires Java 11 or greater
1. In [`.vscode/settings.json`](.vscode/settings.json), set the User Setting, `java.home`, to the correct directory pointing to your JDK 11 directory

### IntelliJ
1. Run `./gradlew idea`
1. Open the `FRC-2024-Public.ipr` file with IntelliJ

### Eclipse
1. Run `./gradlew eclipse`
1. Open Eclipse and go to File > Open Projects from File System...
1. Set the import source to the `FRC-2024-Public` folder then click finish

### Basic Gradle Commands
* Run `./gradlew deploy` to deploy to the robot in Terminal (*nix) or Powershell (Windows)
* Run `./gradlew build` to build the code.  Use the `--info` flag for more details
* Run `./gradlew test` to run all of the JUnit tests

### Simulation
* To simulate the robot run `./gradlew simulateJava`
* You will need to configure your keyboard settings for 1 Xbox Controller.
* To set up 3d visualization in AdvantageScope, import the Advantage_Scope.json.
  * If the JSON import does not work, you need to set up a 3D tab as follows:
    * 3D Poses: Robot (components) : NT: "ComponentsPoseArray"
    * 3D Poses: Robot : NT: "Drive/Viz/Pose3d"
    * 3D Poses: Note : NT: "NoteVisualizer"

## Code Highlights
* Controller Modes

	The robot uses controller modes to modify the controller's controls based on the modal control. The following modes are used:
  - NOT_SPECIFIED - Allows for intaking and shooting against the subwoofer, used if any problem occurs during the match
  - SPEAKER - Ground intake and aims at speaker at all times, adjusting both the turret and hood based on location
  - HP - [Source intake](src/main/java/com/team254/frc2024/command_factories/SuperstructureFactory.java#L230) by raising the elevator, poops following the same logic as POOP mode
  - POOP - Ground intake, [poop pose generated](src/main/java/com/team254/lib/util/PoopTargetFactory.java) based on obstacles and location
	- [Line-Drive](src/main/java/com/team254/lib/util/PoopTargetFactory.java#L80) - when close to poop point without stage in the way
	- [Shallow vs Deep Amp Lob Pass](src/main/java/com/team254/frc2024/RobotContainer.java#L538) - changes location of lob pass based on controller input
  - [CLIMB](src/main/java/com/team254/frc2024/command_factories/SuperstructureFactory.java#L393) - raises climb arms, stages amp and elevator for trap, then climbs

* Branching Auto with Choreo

	In autonomous, the robot uses Choreo and [branching autos](src/main/java/com/team254/frc2024/commands/autocommands/BranchingAutoInterrupt.java) to customize each autonomous based on multiple selectors. At the midline, the robot decides between switching to another note if it did not pick up the current note, scoring the note and going to the next note, or initiating the last action. The following selectors are used before the match:
  - Starting Location - starting location of the robot, including Truss, Speaker, Source, SpeakerCorner, Amp
  - First Action - first action of the robot, including Three Close to Midline, One Close to Midline, Sprint to Midline, and Sprint to Midline No Preload
  - Priority Midline Sequence - sequence of midline notes to target in order, such as ABC, BAC, and EDC. A is the note closest to the Amp, and E is closest to the Source.
  - Last Action - last action of the robot, including Backoff, Score Preload, and Three Close
  - Blocked Notes - Midline notes that the robot should not branch off to - used if another robot in the alliance is targeting other notes


	Choreo is used to create all our paths (Open [.chor file](autopaths.chor) in Choreo), including midline note-to-note branching, scoring, and first and last actions. Paths to midline notes are interrupted when the intake banner sensor is triggered.

* AdvantageScope Simulation

	We used AdvantageScope to simulate subsystem movements and test new software, especially autonomous paths, without the need for the robot. We also visualized logs and live robot telemetry, including note projectile simulation. We imported the robot 3D model with a fully articulating Turret, Hood, Amplifier, and Climber, along with accurate moments of inertia and gear ratios to accurately visualize robot actions. 	 

	We also used AdvantageKit to log all important data to replay after each match. This included vision estimates, robot state values, and subsystem values. This helped to isolate issues after the match, including localization and other robot malfunctions.

* Auto-Aiming to Goal

	Our robot auto-aligns the hood and turret to goal in both Poop mode and Speaker mode. In [poop mode](src/main/java/com/team254/lib/util/ShooterSetpoint.java#L117), we calculate the optimal hood and turret setpoints to reach the target poop pose, considering the constraints of the hood and the desired apex height. In [speaker mode](src/main/java/com/team254/lib/util/ShooterSetpoint.java#L203), we determine the note launch speed based on the distance from target, and use motion and lift compensation to accurately represent the note’s trajectory when the robot is in motion. We wait until all subsystems are [on target](src/main/java/com/team254/frc2024/command_factories/AimFactory.java#L50) for goal before releasing, including the shooter RPS, the turret position, and hood position.

## Package Functions
- [`com.team254.frc2024`](src/main/java/com/team254/frc2024)

	Contains the robot's central functions and holds a class with all numerical constants used throughout the code (see [`Constants.java`](src/main/java/com/team254/frc2024/Constants.java)). For example, the [`RobotContainer`](src/main/java/com/team254/frc2024/RobotContainer.java) class controls all routines depending on the robot mode. In addition, the [`RobotState`](src/main/java/com/team254/frc2024/RobotState.java) class keeps track of the current position of the robot's various frames of reference.

- [`com.team254.frc2024.command_factories`](src/main/java/com/team254/frc2024/command_factories)

	Contains all command factories with methods that call groups of subsystem commands.

- [`com.team254.frc2024.commands`](src/main/java/com/team254/frc2024/commands)

	Contains all autonomous commands and other complex subsystem commands.

- [`com.team254.frc2024.controlboard`](src/main/java/com/team254/frc2024/controlboard)

	Contains code for the driver to use either joysticks with operator gamepad or single gamepad. Also contains modal controls to switch between controller modes.

- [`com.team254.frc2024.simulation`](src/main/java/com/team254/frc2024/simulation)

	Contains code for simulated robot state, including the note state and robot position.

- [`com.team254.frc2024.subsystems`](src/main/java/com/team254/frc2024/subsystems)

	Contains the code for all subsystems. Each subsystem extends [`ServoMotorSubsystem`](src/main/java/com/team254/lib/subsystems/ServoMotorSubsystem.java) and includes an simulation and hardware interface for I/O operations.

- [`com.team254.lib.auto`](src/main/java/com/team254/lib/auto)

	Contains auto utility class to generate autonomous trajectories and commands.

- [`com.team254.lib.ctre.swerve`](src/main/java/com/team254/lib/ctre/swerve)

	Contains forked CTRE swerve classes for custom drive configs.

- [`com.team254.lib.drivers`](src/main/java/com/team254/lib/drivers)

	Contains CAN device ID class, pairing device number and bus name.

- [`com.team254.lib.limelight`](src/main/java/com/team254/lib/limelight)

	Contains Limelight helpers class for reading from NetworkTables.

- [`com.team254.lib.motion`](src/main/java/com/team254/lib/motion)

	Contains all motion profiling code used for autonomous driving. Trapezoidal motion profiles are used for smooth acceleration and minimal slip.

- [`com.team254.lib.loops`](src/main/java/com/team254/lib/loops)

	Contains a thread loop for running a set of callbacks at a Hz driven by the update frequency of CTRE Status Signals.

- [`com.team254.lib.pathplanner`](src/main/java/com/team254/lib/pathplanner)

	Contains forked Pathplanner classes improve PID controller, velocity feedback, and acceleration feedforward.

- [`com.team254.lib.subsystems`](src/main/java/com/team254/lib/subsystems)

	Contains servo motor subsystem template to be extended by all subsystems.

- [`com.team254.lib.time`](src/main/java/com/team254/lib/time)

	Contains robot time.

- [`com.team254.lib.util`](src/main/java/com/team254/lib/util)

	Contains a collection of assorted utilities classes used in the robot code. Check each file for more information.

## Variable Naming Conventions
- k*** (i.e. `kDriveWheelbaseMeters`): Final constants, especially those found in the [`Constants.java`](src/main/java/com/team254/frc2024/Constants.java) file
