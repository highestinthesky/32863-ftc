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
        data.put(24.5, 2000);
        data.put(30, 2400);
        data.put(36, 2800);
        data.put(42, 3300);
        data.put(48, 3700);
        data.put(54, 4100);
        data.put(60, 4500);

        return data;
    }
}