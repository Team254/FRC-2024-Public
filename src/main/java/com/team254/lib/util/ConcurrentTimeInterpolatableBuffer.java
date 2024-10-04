// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team254.lib.util;

import java.util.Map.Entry;
import java.util.Optional;
import java.util.concurrent.ConcurrentNavigableMap;
import java.util.concurrent.ConcurrentSkipListMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.Interpolator;

/**
 * A concurrent version of WPIlib's TimeInterpolatableBuffer class to avoid the
 * need for explicit synchronization
 * in our robot code.
 *
 * @param <T> The type stored in this buffer.
 */
public final class ConcurrentTimeInterpolatableBuffer<T> {
    private final double m_historySize;
    private final Interpolator<T> m_interpolatingFunc;
    private final ConcurrentNavigableMap<Double, T> m_pastSnapshots = new ConcurrentSkipListMap<>();

    private ConcurrentTimeInterpolatableBuffer(Interpolator<T> interpolateFunction, double historySizeSeconds) {
        this.m_historySize = historySizeSeconds;
        this.m_interpolatingFunc = interpolateFunction;
    }

    /**
     * Create a new TimeInterpolatableBuffer.
     *
     * @param interpolateFunction The function used to interpolate between values.
     * @param historySizeSeconds  The history size of the buffer.
     * @param <T>                 The type of data to store in the buffer.
     * @return The new TimeInterpolatableBuffer.
     */
    public static <T> ConcurrentTimeInterpolatableBuffer<T> createBuffer(
            Interpolator<T> interpolateFunction, double historySizeSeconds) {
        return new ConcurrentTimeInterpolatableBuffer<>(interpolateFunction, historySizeSeconds);
    }

    /**
     * Create a new TimeInterpolatableBuffer that stores a given subclass of
     * {@link Interpolatable}.
     *
     * @param historySizeSeconds The history size of the buffer.
     * @param <T>                The type of {@link Interpolatable} to store in the
     *                           buffer.
     * @return The new TimeInterpolatableBuffer.
     */
    public static <T extends Interpolatable<T>> ConcurrentTimeInterpolatableBuffer<T> createBuffer(
            double historySizeSeconds) {
        return new ConcurrentTimeInterpolatableBuffer<>(Interpolatable::interpolate, historySizeSeconds);
    }

    /**
     * Create a new TimeInterpolatableBuffer to store Double values.
     *
     * @param historySizeSeconds The history size of the buffer.
     * @return The new TimeInterpolatableBuffer.
     */
    public static ConcurrentTimeInterpolatableBuffer<Double> createDoubleBuffer(double historySizeSeconds) {
        return new ConcurrentTimeInterpolatableBuffer<>(MathUtil::interpolate, historySizeSeconds);
    }

    /**
     * Add a sample to the buffer.
     *
     * @param timeSeconds The timestamp of the sample.
     * @param sample      The sample object.
     */
    public void addSample(double timeSeconds, T sample) {
        m_pastSnapshots.put(timeSeconds, sample);
        cleanUp(timeSeconds);
    }

    /**
     * Removes samples older than our current history size.
     *
     * @param time The current timestamp.
     */
    private void cleanUp(double time) {
        m_pastSnapshots.headMap(time - m_historySize, false).clear();
    }

    /** Clear all old samples. */
    public void clear() {
        m_pastSnapshots.clear();
    }

    /**
     * Sample the buffer at the given time. If the buffer is empty, an empty
     * Optional is returned.
     *
     * @param timeSeconds The time at which to sample.
     * @return The interpolated value at that timestamp or an empty Optional.
     */
    public Optional<T> getSample(double timeSeconds) {
        if (m_pastSnapshots.isEmpty()) {
            return Optional.empty();
        }

        // Special case for when the requested time is the same as a sample
        var nowEntry = m_pastSnapshots.get(timeSeconds);
        if (nowEntry != null) {
            return Optional.of(nowEntry);
        }

        var bottomBound = m_pastSnapshots.floorEntry(timeSeconds);
        var topBound = m_pastSnapshots.ceilingEntry(timeSeconds);

        // Return null if neither sample exists, and the opposite bound if the other is
        // null
        if (topBound == null && bottomBound == null) {
            return Optional.empty();
        } else if (topBound == null) {
            return Optional.of(bottomBound.getValue());
        } else if (bottomBound == null) {
            return Optional.of(topBound.getValue());
        } else {
            // Otherwise, interpolate. Because T is between [0, 1], we want the ratio of
            // (the difference
            // between the current time and bottom bound) and (the difference between top
            // and bottom
            // bounds).
            return Optional.of(
                    m_interpolatingFunc.interpolate(
                            bottomBound.getValue(),
                            topBound.getValue(),
                            (timeSeconds - bottomBound.getKey()) / (topBound.getKey() - bottomBound.getKey())));
        }
    }

    public Entry<Double, T> getLatest() {
        return m_pastSnapshots.lastEntry();
    }

    /**
     * Grant access to the internal sample buffer. Used in Pose Estimation to replay
     * odometry inputs
     * stored within this buffer.
     *
     * @return The internal sample buffer.
     */
    public ConcurrentNavigableMap<Double, T> getInternalBuffer() {
        return m_pastSnapshots;
    }
}
