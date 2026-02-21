// this class is completely currently AI generated

package org.firstinspires.ftc.teamcode.shooter;

import java.util.NavigableMap;
import java.util.TreeMap;

/**
 * Stores shooter calibration data as (distanceInches -> targetVelocity).
 * Uses linear interpolation between nearest points.
 * Distance unit: inches
 * Velocity unit: whatever you choose to command (recommended: ticks/sec to match DcMotorEx.setVelocity()).
 */
public class ShooterData {

    private final TreeMap<Double, Double> distanceToVelocity = new TreeMap<>();

    public ShooterData() {}

    /** Add / overwrite a calibration point. */
    public void put(double distanceInches, double targetVelocity) {
        distanceToVelocity.put(distanceInches, targetVelocity);
    }

    /** Remove a calibration point at an exact distance key (if present). */
    public void remove(double distanceInches) {
        distanceToVelocity.remove(distanceInches);
    }

    /** Clear all calibration points. */
    public void clear() {
        distanceToVelocity.clear();
    }

    /** @return true if there are no points stored */
    public boolean isEmpty() {
        return distanceToVelocity.isEmpty();
    }

    /** @return view of the internal map (useful for telemetry/debug) */
    public NavigableMap<Double, Double> asMap() {
        return distanceToVelocity;
    }

    /**
     * Get interpolated velocity for a given distance in inches.
     * - Clamps to first/last entry outside range.
     * - Linearly interpolates between nearest two points inside range.
     */
    public double getTargetVelocity(double distanceInches) {
        if (distanceToVelocity.isEmpty()) return 0.0;

        Double loKey = distanceToVelocity.floorKey(distanceInches);
        Double hiKey = distanceToVelocity.ceilingKey(distanceInches);

        // Clamp outside range
        if (loKey == null) return distanceToVelocity.firstEntry().getValue();
        if (hiKey == null) return distanceToVelocity.lastEntry().getValue();

        // Exact key match
        if (loKey.equals(hiKey)) return distanceToVelocity.get(loKey);

        double loVal = distanceToVelocity.get(loKey);
        double hiVal = distanceToVelocity.get(hiKey);

        // Linear interpolation
        double t = (distanceInches - loKey) / (hiKey - loKey);
        return loVal + t * (hiVal - loVal);
    }

    /**
     * Optional: convenience builder for a "default" dataset.
     * Replace these with YOUR tuned values.
     */
    public static ShooterData defaultTuned() {
        ShooterData data = new ShooterData();

        // TODO: Replace with real tuned points (distance inches, velocity units you use).
        data.put(24, 1600);
        data.put(30, 1750);
        data.put(36, 1850);
        data.put(42, 1950);
        data.put(48, 2050);
        data.put(54, 2150);
        data.put(60, 2250);

        return data;
    }
}