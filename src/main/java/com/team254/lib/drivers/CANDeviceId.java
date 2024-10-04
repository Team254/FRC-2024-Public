package com.team254.lib.drivers;

public class CANDeviceId {
    private final int deviceNumber;
    private final String bus;

    public CANDeviceId(int deviceNumber, String bus) {
        this.deviceNumber = deviceNumber;
        this.bus = bus;
    }

    // Use the default bus name (empty string).
    public CANDeviceId(int deviceNumber) {
        this(deviceNumber, "");
    }

    public int getDeviceNumber() {
        return deviceNumber;
    }

    public String getBus() {
        return bus;
    }

    public boolean equals(CANDeviceId other) {
        return other.deviceNumber == deviceNumber && other.bus == bus;
    }
}