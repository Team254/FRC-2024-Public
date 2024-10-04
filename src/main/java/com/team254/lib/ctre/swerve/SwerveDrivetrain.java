/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package com.team254.lib.ctre.swerve;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Consumer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team254.lib.util.MathHelpers;

import static com.team254.lib.ctre.swerve.SwerveRequest.SwerveControlRequestParameters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;

/**
 * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API.
 * <p>
 * This class handles the kinematics, configuration, and odometry of a
 * swerve drive utilizing CTR Electronics devices. We recommend
 * that users use the Swerve Mechanism Generator in Tuner X to create
 * a template project that demonstrates how to use this class.
 * <p>
 * This class will construct the hardware devices internally, so the user
 * only specifies the constants (IDs, PID gains, gear ratios, etc).
 * Getters for these hardware devices are available.
 * <p>
 * If using the generator, the order in which modules are constructed is
 * Front Left, Front Right, Back Left, Back Right. This means if you need
 * the Back Left module, call {@code getModule(2);} to get the 3rd index
 * (0-indexed) module, corresponding to the Back Left module.
 */
public class SwerveDrivetrain {
    protected final boolean IsOnCANFD;

    protected final double UpdateFrequency;
    protected final int ModuleCount;
    protected final SwerveModule[] Modules;

    protected final Pigeon2 m_pigeon2;
    protected final StatusSignal<Double> m_yawGetter;
    protected final StatusSignal<Double> m_angularVelocity;

    protected SwerveDriveKinematics m_kinematics;
    protected SwerveDrivePoseEstimator m_odometry;
    protected SwerveModulePosition[] m_modulePositions;
    protected SwerveModuleState[] m_moduleStates;
    protected Translation2d[] m_moduleLocations;
    protected OdometryThread m_odometryThread;
    protected Rotation2d m_fieldRelativeOffset;
    protected Rotation2d m_operatorForwardDirection;

    protected SwerveRequest m_requestToApply = new SwerveRequest.Idle();
    protected SwerveControlRequestParameters m_requestParameters = new SwerveControlRequestParameters();

    protected final ReadWriteLock m_stateLock = new ReentrantReadWriteLock();

    protected final SimSwerveDrivetrain m_simDrive;

    /**
     * Plain-Old-Data class holding the state of the swerve drivetrain.
     * This encapsulates most data that is relevant for telemetry or
     * decision-making from the Swerve Drive.
     */
    public static class SwerveDriveState {
        /** Number of successful data acquisitions */
        public int SuccessfulDaqs;
        /** Number of failed data acquisitions */
        public int FailedDaqs;
        /** The current pose of the robot */
        public Pose2d Pose;
        /** The current velocity of the robot */
        public ChassisSpeeds speeds;
        /** The current module states */
        public SwerveModuleState[] ModuleStates;
        /** The target module states */
        public SwerveModuleState[] ModuleTargets;
        /** The measured odometry update period, in seconds */
        public double OdometryPeriod;

        public SwerveDriveState clone() {
            SwerveDriveState other = new SwerveDriveState();
            other.SuccessfulDaqs = this.SuccessfulDaqs;
            other.FailedDaqs = this.FailedDaqs;
            other.Pose = this.Pose;
            other.speeds = this.speeds;
            other.ModuleStates = this.ModuleStates;
            other.ModuleTargets = this.ModuleTargets;
            other.OdometryPeriod = this.OdometryPeriod;
            return other;
        }
    }

    protected Consumer<SwerveDriveState> m_telemetryFunction = null;
    protected final SwerveDriveState m_cachedState = new SwerveDriveState();

    /* Perform swerve module updates in a separate thread to minimize latency */
    public class OdometryThread {
        protected static final int START_THREAD_PRIORITY = 2; // Testing shows 1 (minimum realtime) is sufficient for
                                                              // tighter
                                                              // odometry loops.
                                                              // If the odometry period is far away from the desired
                                                              // frequency,
                                                              // increasing this may help

        protected final Thread m_thread;
        protected volatile boolean m_running = false;

        protected final BaseStatusSignal[] m_allSignals;

        protected final MedianFilter peakRemover = new MedianFilter(3);
        protected final LinearFilter lowPass = LinearFilter.movingAverage(50);
        protected double lastTime = 0;
        protected double currentTime = 0;
        protected double averageLoopTime = 0;
        protected int SuccessfulDaqs = 0;
        protected int FailedDaqs = 0;

        protected int lastThreadPriority = START_THREAD_PRIORITY;
        protected volatile int threadPriorityToSet = START_THREAD_PRIORITY;

        public OdometryThread() {
            m_thread = new Thread(this::run);
            /*
             * Mark this thread as a "daemon" (background) thread
             * so it doesn't hold up program shutdown
             */
            m_thread.setDaemon(true);

            /* 4 signals for each module + 2 for Pigeon2 */
            m_allSignals = new BaseStatusSignal[(ModuleCount * 4) + 2];
            for (int i = 0; i < ModuleCount; ++i) {
                var signals = Modules[i].getSignals();
                m_allSignals[(i * 4) + 0] = signals[0];
                m_allSignals[(i * 4) + 1] = signals[1];
                m_allSignals[(i * 4) + 2] = signals[2];
                m_allSignals[(i * 4) + 3] = signals[3];
            }
            m_allSignals[m_allSignals.length - 2] = m_yawGetter;
            m_allSignals[m_allSignals.length - 1] = m_angularVelocity;
        }

        /**
         * Starts the odometry thread.
         */
        public void start() {
            m_running = true;
            m_thread.start();
        }

        /**
         * Stops the odometry thread.
         */
        public void stop() {
            stop(0);
        }

        /**
         * Stops the odometry thread with a timeout.
         *
         * @param millis The time to wait in milliseconds
         */
        public void stop(long millis) {
            m_running = false;
            try {
                m_thread.join(millis);
            } catch (final InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
        }

        public void run() {
            /* Make sure all signals update at the correct update frequency */
            BaseStatusSignal.setUpdateFrequencyForAll(UpdateFrequency, m_allSignals);
            Threads.setCurrentThreadPriority(true, START_THREAD_PRIORITY);

            /* Run as fast as possible, our signals will control the timing */
            while (m_running) {
                /* Synchronously wait for all signals in drivetrain */
                /* Wait up to twice the period of the update frequency */
                StatusCode status;
                if (IsOnCANFD) {
                    status = BaseStatusSignal.waitForAll(2.0 / UpdateFrequency, m_allSignals);
                } else {
                    /* Wait for the signals to update */
                    Timer.delay(1.0 / UpdateFrequency);
                    status = BaseStatusSignal.refreshAll(m_allSignals);
                }

                try {
                    m_stateLock.writeLock().lock();

                    lastTime = currentTime;
                    currentTime = Utils.getCurrentTimeSeconds();
                    /*
                     * We don't care about the peaks, as they correspond to GC events, and we want
                     * the period generally low passed
                     */
                    averageLoopTime = lowPass.calculate(peakRemover.calculate(currentTime - lastTime));

                    /* Get status of first element */
                    if (status.isOK()) {
                        SuccessfulDaqs++;
                    } else {
                        FailedDaqs++;
                    }

                    /* Now update odometry */
                    /* Keep track of the change in azimuth rotations */
                    for (int i = 0; i < ModuleCount; ++i) {
                        m_modulePositions[i] = Modules[i].getPosition(false);
                        m_moduleStates[i] = Modules[i].getCurrentState();
                    }
                    double yawDegrees = BaseStatusSignal.getLatencyCompensatedValue(
                            m_yawGetter, m_angularVelocity);

                    /* Keep track of previous and current pose to account for the carpet vector */
                    m_odometry.update(Rotation2d.fromDegrees(yawDegrees), m_modulePositions);

                    ChassisSpeeds speeds = m_kinematics.toChassisSpeeds(m_moduleStates);

                    /* And now that we've got the new odometry, update the controls */
                    m_requestParameters.currentPose = m_odometry.getEstimatedPosition()
                            .relativeTo(MathHelpers.pose2dFromRotation(m_fieldRelativeOffset));
                    m_requestParameters.kinematics = m_kinematics;
                    m_requestParameters.swervePositions = m_moduleLocations;
                    m_requestParameters.currentChassisSpeed = speeds;
                    m_requestParameters.timestamp = currentTime;
                    m_requestParameters.updatePeriod = 1.0 / UpdateFrequency;
                    m_requestParameters.operatorForwardDirection = m_operatorForwardDirection;

                    m_requestToApply.apply(m_requestParameters, Modules);

                    /* Update our cached state with the newly updated data */
                    m_cachedState.FailedDaqs = FailedDaqs;
                    m_cachedState.SuccessfulDaqs = SuccessfulDaqs;
                    m_cachedState.Pose = m_odometry.getEstimatedPosition();
                    m_cachedState.speeds = speeds;
                    m_cachedState.OdometryPeriod = averageLoopTime;

                    if (m_cachedState.ModuleStates == null) {
                        m_cachedState.ModuleStates = new SwerveModuleState[Modules.length];
                    }
                    if (m_cachedState.ModuleTargets == null) {
                        m_cachedState.ModuleTargets = new SwerveModuleState[Modules.length];
                    }
                    for (int i = 0; i < Modules.length; ++i) {
                        m_cachedState.ModuleStates[i] = Modules[i].getCurrentState();
                        m_cachedState.ModuleTargets[i] = Modules[i].getTargetState();
                    }

                    if (m_telemetryFunction != null) {
                        /* Log our state */
                        m_telemetryFunction.accept(m_cachedState);
                    }
                } finally {
                    m_stateLock.writeLock().unlock();
                }

                /**
                 * This is inherently synchronous, since lastThreadPriority
                 * is only written here and threadPriorityToSet is only read here
                 */
                if (threadPriorityToSet != lastThreadPriority) {
                    Threads.setCurrentThreadPriority(true, threadPriorityToSet);
                    lastThreadPriority = threadPriorityToSet;
                }
            }
        }

        public boolean odometryIsValid() {
            return SuccessfulDaqs > 2; // Wait at least 3 daqs before saying the odometry is valid
        }

        /**
         * Sets the DAQ thread priority to a real time priority under the specified
         * priority level
         *
         * @param priority Priority level to set the DAQ thread to.
         *                 This is a value between 0 and 99, with 99 indicating higher
         *                 priority and 0 indicating lower priority.
         */
        public void setThreadPriority(int priority) {
            threadPriorityToSet = priority;
        }
    }

    protected boolean checkIsOnCanFD(String canbusName) {
        return CANBus.isNetworkFD(canbusName);
    }

    /**
     * Constructs a CTRSwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so user should not construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param driveTrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public SwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        this(driveTrainConstants, 0, modules);
    }

    /**
     * Constructs a CTRSwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so user should not construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param driveTrainConstants     Drivetrain-wide constants for the swerve drive
     * @param OdometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified, this is 250 Hz on CAN FD, and
     *                                100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public SwerveDrivetrain(
            SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        this(driveTrainConstants, OdometryUpdateFrequency,
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.9, 0.9, 0.9),
                modules);
    }

    /**
     * Constructs a CTRSwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so user should not construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param driveTrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param OdometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified, this is 250 Hz on CAN FD, and
     *                                  100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     * @param modules                   Constants for each specific module
     */
    public SwerveDrivetrain(
            SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants... modules) {
        IsOnCANFD = checkIsOnCanFD(driveTrainConstants.CANbusName);

        if (OdometryUpdateFrequency == 0) {
            UpdateFrequency = IsOnCANFD ? 250 : 100;
        } else {
            UpdateFrequency = OdometryUpdateFrequency;
        }
        ModuleCount = modules.length;

        m_pigeon2 = new Pigeon2(driveTrainConstants.Pigeon2Id, driveTrainConstants.CANbusName);
        m_yawGetter = m_pigeon2.getYaw().clone();
        m_angularVelocity = m_pigeon2.getAngularVelocityZWorld().clone();

        Modules = new SwerveModule[ModuleCount];
        m_modulePositions = new SwerveModulePosition[ModuleCount];
        m_moduleStates = new SwerveModuleState[ModuleCount];
        m_moduleLocations = new Translation2d[ModuleCount];

        int iteration = 0;
        for (SwerveModuleConstants module : modules) {
            Modules[iteration] = new SwerveModule(module, driveTrainConstants.CANbusName);
            m_moduleLocations[iteration] = new Translation2d(module.LocationX, module.LocationY);
            m_modulePositions[iteration] = Modules[iteration].getPosition(true);
            m_moduleStates[iteration] = Modules[iteration].getCurrentState();

            iteration++;
        }
        m_kinematics = new SwerveDriveKinematics(m_moduleLocations);
        m_odometry = new SwerveDrivePoseEstimator(m_kinematics, new Rotation2d(), m_modulePositions, new Pose2d(),
                odometryStandardDeviation, visionStandardDeviation);

        m_fieldRelativeOffset = new Rotation2d();
        m_operatorForwardDirection = new Rotation2d();

        m_simDrive = new SimSwerveDrivetrain(m_moduleLocations, m_pigeon2, driveTrainConstants, modules);

        m_odometryThread = new OdometryThread();
        m_odometryThread.start();
    }

    /**
     * Gets a reference to the data acquisition thread.
     *
     * @return DAQ thread
     */
    public OdometryThread getDaqThread() {
        return m_odometryThread;
    }

    /**
     * Applies the specified control request to this swerve drivetrain.
     *
     * @param request Request to apply
     */
    public void setControl(SwerveRequest request) {
        try {
            m_stateLock.writeLock().lock();

            m_requestToApply = request;
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Configures the neutral mode to use for all modules' drive motors.
     *
     * @param neutralMode The drive motor neutral mode
     * @return Status code of the first failed config call, or OK if all succeeded
     */
    public StatusCode configNeutralMode(NeutralModeValue neutralMode) {
        var status = StatusCode.OK;
        for (var module : Modules) {
            var moduleStatus = module.configNeutralMode(neutralMode);
            if (status.isOK()) {
                status = moduleStatus;
            }
        }
        return status;
    }

    /**
     * Zero's this swerve drive's odometry entirely.
     * <p>
     * This will zero the entire odometry, and place the robot at 0,0
     */
    public void tareEverything() {
        try {
            m_stateLock.writeLock().lock();

            for (int i = 0; i < ModuleCount; ++i) {
                Modules[i].resetPosition();
                m_modulePositions[i] = Modules[i].getPosition(true);
            }
            m_odometry.resetPosition(Rotation2d.fromDegrees(m_yawGetter.getValue()), m_modulePositions, new Pose2d());
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Takes the current orientation of the robot and makes it X forward for
     * field-relative
     * maneuvers.
     */
    public void seedFieldRelative() {
        try {
            m_stateLock.writeLock().lock();

            m_fieldRelativeOffset = getState().Pose.getRotation();
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Takes the {@link SwerveRequest.ForwardReference#RedAlliance} perpective
     * direction
     * and treats it as the forward direction for
     * {@link SwerveRequest.ForwardReference#OperatorPerspective}.
     * <p>
     * If the operator is in the Blue Alliance Station, this should be 0 degrees.
     * If the operator is in the Red Alliance Station, this should be 180 degrees.
     *
     * @param fieldDirection Heading indicating which direction is forward from
     *                       the {@link SwerveRequest.ForwardReference#RedAlliance}
     *                       perspective.
     */
    public void setOperatorPerspectiveForward(Rotation2d fieldDirection) {
        m_operatorForwardDirection = fieldDirection;
    }

    /**
     * Takes the specified location and makes it the current pose for
     * field-relative maneuvers
     *
     * @param location Pose to make the current pose at.
     */
    public void seedFieldRelative(Pose2d location) {
        try {
            m_stateLock.writeLock().lock();

            m_odometry.resetPosition(Rotation2d.fromDegrees(m_yawGetter.getValue()), m_modulePositions, location);
            /*
             * We need to update our cached pose immediately so that race conditions don't
             * happen
             */
            m_cachedState.Pose = location;
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Check if the odometry is currently valid
     *
     * @return True if odometry is valid
     */
    public boolean odometryIsValid() {
        try {
            m_stateLock.readLock().lock();

            return m_odometryThread.odometryIsValid();
        } finally {
            m_stateLock.readLock().unlock();
        }
    }

    /**
     * Get a reference to the module at the specified index.
     * The index corresponds to the module described in the constructor
     *
     * @param index Which module to get
     * @return Reference to SwerveModule
     */
    public SwerveModule getModule(int index) {
        if (index >= Modules.length)
            return null;
        return Modules[index];
    }

    /**
     * Gets the current state of the swerve drivetrain.
     *
     * @return Current state of the drivetrain
     */
    public SwerveDriveState getState() {
        try {
            m_stateLock.readLock().lock();

            return m_cachedState;
        } finally {
            m_stateLock.readLock().unlock();
        }
    }

    /**
     * Gets the current orientation of the robot as a {@link Rotation3d} from
     * the Pigeon 2 quaternion values.
     *
     * @return The robot orientation as a {@link Rotation3d}
     */
    public Rotation3d getRotation3d() {
        return m_pigeon2.getRotation3d();
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * <p>
     * This method can be called as infrequently as you want, as long as you are
     * calling {@link
     * SwerveDrivePoseEstimator#update} every loop.
     *
     * <p>
     * To promote stability of the pose estimate and make it robust to bad vision
     * data, we
     * recommend only adding vision measurements that are already within one meter
     * or so of the
     * current pose estimate.
     *
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue
     * to apply to future measurements until a subsequent call to {@link
     * SwerveDrivePoseEstimator#setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds. Note that if you
     *                                 don't use your own time source by calling
     *                                 {@link
     *                                 SwerveDrivePoseEstimator#updateWithTime(double,Rotation2d,SwerveModulePosition[])},
     *                                 then
     *                                 you must use a timestamp with an epoch since
     *                                 FPGA startup (i.e., the epoch of this
     *                                 timestamp is the same epoch as
     *                                 {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}).
     *                                 This means that you should use
     *                                 {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}
     *                                 as
     *                                 your time source in this case.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement (x position
     *                                 in meters, y position in meters, and heading
     *                                 in radians). Increase these numbers to trust
     *                                 the vision pose measurement less.
     */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        try {
            m_stateLock.writeLock().lock();
            m_odometry.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * <p>
     * This method can be called as infrequently as you want, as long as you are
     * calling {@link
     * SwerveDrivePoseEstimator#update} every loop.
     *
     * <p>
     * To promote stability of the pose estimate and make it robust to bad vision
     * data, we
     * recommend only adding vision measurements that are already within one meter
     * or so of the
     * current pose estimate.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds. Note that if you
     *                              don't use your own time source by calling {@link
     *                              SwerveDrivePoseEstimator#updateWithTime(double,Rotation2d,SwerveModulePosition[])}
     *                              then you
     *                              must use a timestamp with an epoch since FPGA
     *                              startup (i.e., the epoch of this timestamp is
     *                              the same epoch as
     *                              {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}.)
     *                              This means that
     *                              you should use
     *                              {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}
     *                              as your time source
     *                              or sync the epochs.
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        try {
            m_stateLock.writeLock().lock();
            m_odometry.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Sets the pose estimator's trust of global measurements. This might be used to
     * change trust in
     * vision measurements after the autonomous period, or to change trust as
     * distance to a vision
     * target increases.
     *
     * @param visionMeasurementStdDevs Standard deviations of the vision
     *                                 measurements. Increase these
     *                                 numbers to trust global measurements from
     *                                 vision less. This matrix is in the form [x,
     *                                 y,
     *                                 theta]ᵀ, with units in meters and radians.
     */
    public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
        try {
            m_stateLock.writeLock().lock();
            m_odometry.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Updates all the simulation state variables for this
     * drivetrain class. User provides the update variables for the simulation.
     *
     * @param dtSeconds     time since last update call
     * @param supplyVoltage voltage as seen at the motor controllers
     */
    public void updateSimState(double dtSeconds, double supplyVoltage) {
        m_simDrive.update(dtSeconds, supplyVoltage, Modules);
    }

    /**
     * Register the specified lambda to be executed whenever our SwerveDriveState
     * function
     * is updated in our odometry thread.
     * <p>
     * It is imperative that this function is cheap, as it will be executed along
     * with
     * the odometry call, and if this takes a long time, it may negatively impact
     * the odometry of this stack.
     * <p>
     * This can also be used for logging data if the function performs logging
     * instead of telemetry
     *
     * @param telemetryFunction Function to call for telemetry or logging
     */
    public void registerTelemetry(Consumer<SwerveDriveState> telemetryFunction) {
        try {
            m_stateLock.writeLock().lock();
            m_telemetryFunction = telemetryFunction;
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Gets this drivetrain's Pigeon 2 reference.
     * <p>
     * This should be used only to access signals and change configurations that the
     * swerve drivetrain does not configure itself.
     *
     * @return This drivetrain's Pigeon 2 reference
     */
    public Pigeon2 getPigeon2() {
        return m_pigeon2;
    }
}
