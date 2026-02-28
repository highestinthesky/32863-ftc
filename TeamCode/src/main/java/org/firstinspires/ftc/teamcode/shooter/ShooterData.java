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

        // Placeholder baseline (intentionally conservative but higher than previous set).
        // Tune on-robot with LimelightFlywheelDistanceTuner.
        data.put(10.0, 2200);
        data.put(14.0, 2400);
        data.put(18.0, 2650);
        data.put(22.0, 2900);
        data.put(24.5, 3050);
        data.put(30.0, 3450);
        data.put(36.0, 3900);
        data.put(42.0, 4350);
        data.put(48.0, 4750);
        data.put(54.0, 5150);
        data.put(60.0, 5550);

        return data;
    }
}
