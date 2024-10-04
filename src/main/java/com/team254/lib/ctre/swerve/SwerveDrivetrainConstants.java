/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package com.team254.lib.ctre.swerve;

/**
 * Common constants for a swerve drivetrain.
 */
public class SwerveDrivetrainConstants {
    /** CAN ID of the Pigeon2 on the drivetrain. */
    public int Pigeon2Id = 0;
    /**
     * Name of the CAN bus the swerve drive is on.
     * Possible CAN bus strings are:
     * <ul>
     * <li>"rio" for the native roboRIO CAN bus
     * <li>CANivore name or serial number
     * <li>"*" for any CANivore seen by the program
     * </ul>
     */
    public String CANbusName = "rio";

    /**
     * Sets the CAN ID of the Pigeon2 on the drivetrain.
     *
     * @param id CAN ID of the Pigeon2 on the drivetrain
     * @return this object
     */
    public SwerveDrivetrainConstants withPigeon2Id(int id) {
        this.Pigeon2Id = id;
        return this;
    }

    /**
     * Sets the name of the CAN bus the swerve drive is on.
     * Possible CAN bus strings are:
     * <ul>
     * <li>"rio" for the native roboRIO CAN bus
     * <li>CANivore name or serial number
     * <li>"*" for any CANivore seen by the program
     * </ul>
     *
     * @param name Name of the CAN bus the swerve drive is on
     * @return this object
     */
    public SwerveDrivetrainConstants withCANbusName(String name) {
        this.CANbusName = name;
        return this;
    }
}
