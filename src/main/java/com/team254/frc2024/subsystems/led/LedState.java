package com.team254.frc2024.subsystems.led;

/**
 * The {@code LedState} class represents the RGB color state of an LED.
 * It provides predefined static instances for common colors,
 * along with utility methods for handling and comparing LED states.
 * This class supports custom RGB values for controlling LED colors in hardware.
 */

public class LedState {
    public static final LedState kRed = new LedState(255, 0, 0);
    public static final LedState kBlue = new LedState(0, 0, 255);
    public static final LedState kCyan = new LedState(0, 255, 255);
    public static final LedState kGreen = new LedState(0, 255, 0);
    public static final LedState kYellow = new LedState(255, 215, 0);
    public static final LedState kOrange = new LedState(255, 80, 0);
    public static final LedState kPurple = new LedState(255, 0, 255);
    public static final LedState kWhite = new LedState(255, 255, 255);

    public static final LedState kOff = new LedState(0, 0, 0); // No Color
    public static final LedState kLowBattery = kRed;
    public static final LedState kGoodBattery = kGreen;
    public static final LedState[] kRainbow = { kWhite, kWhite, kWhite, kWhite, kWhite, kWhite, kWhite, kWhite, kOff,
            kRed, kOrange, kYellow, kGreen, kCyan, kBlue, kPurple, kOff, kOff };

    public int blue;
    public int green;
    public int red;

    public LedState() {
        blue = 0;
        green = 0;
        red = 0;
    }

    public LedState(int r, int g, int b) {
        blue = b;
        green = g;
        red = r;
    }

    public void copyFrom(LedState other) {
        this.blue = other.blue;
        this.green = other.green;
        this.red = other.red;
    }

    @Override
    public boolean equals(Object other) {
        if (other == null) {
            return false;
        }

        if (other.getClass() != this.getClass()) {
            return false;
        }

        LedState s = (LedState) other;
        return this.blue == s.blue && this.red == s.red && this.green == s.green;
    }

    @Override
    public String toString() {
        return "#" + Integer.toHexString(red) + Integer.toHexString(green) + Integer.toHexString(blue);
    }
}