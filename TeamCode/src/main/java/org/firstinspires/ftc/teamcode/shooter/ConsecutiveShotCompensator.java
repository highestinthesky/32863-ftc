package org.firstinspires.ftc.teamcode.shooter;

/**
 * Scaffold for future flywheel compensation tuning between consecutive shots.
 * Currently disabled by default and returns the base velocity unchanged.
 */
public class ConsecutiveShotCompensator {
    private final ShotControlConfig config;
    private String status = "Disabled";

    public ConsecutiveShotCompensator(ShotControlConfig config) {
        this.config = config;
        if (config != null && config.enableConsecutiveCompensation) {
            status = "Enabled (TODO tune)";
        }
    }

    public double apply(double baseVelocity, double leftVelocity, double rightVelocity) {
        if (config == null || !config.enableConsecutiveCompensation) {
            status = "Disabled";
            return baseVelocity;
        }

        // TODO (HUGE): tune this live on robot.
        // Suggested approach:
        // 1) Detect sudden velocity dip immediately after ring feed.
        // 2) Add temporary boost to target velocity.
        // 3) Remove boost once wheel speed recovers.
        double averageVelocity = (Math.abs(leftVelocity) + Math.abs(rightVelocity)) * 0.5;
        double drop = baseVelocity - averageVelocity;
        if (drop > config.compensationVelocityDropThreshold) {
            status = "Compensating";
            return baseVelocity + config.compensationBoostVelocity;
        }

        status = "Holding";
        return baseVelocity;
    }

    public String getStatus() {
        return status;
    }
}
