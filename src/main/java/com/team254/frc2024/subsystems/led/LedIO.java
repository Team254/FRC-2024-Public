package com.team254.frc2024.subsystems.led;

/**
 * Interface for LED subsystem IO operations.
 * This interface defines the methods for reading and writing LED states
 * as well as abstract methods for writing LED pixels to the LEDS on the robot.
 */

public interface LedIO {
    class LedInputs {
    }

    default void readInputs(LedIO.LedInputs inputs) {
    };

    default void update(final LedIO.LedInputs inputs) {
    };

    void writePixels(LedState state);

    void writePixels(LedState[] states);
}
