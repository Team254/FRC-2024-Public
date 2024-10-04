package com.team254.lib.loops;

import com.ctre.phoenix6.BaseStatusSignal;

import java.util.List;

/**
 * Interface for registering into the StatusSignalLoop
 */
public interface IStatusSignalLoop {
    List<BaseStatusSignal> getStatusSignals();

    void onLoop();
}
