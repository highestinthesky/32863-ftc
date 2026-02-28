package org.firstinspires.ftc.teamcode.shooter;

import org.firstinspires.ftc.teamcode.functions;

public class VisionShooterTargeting {
    private final functions.MegaTag2Prep megaTag2;
    private final ShooterData shooterData;
    private final int goalTagId;

    private boolean targetAvailable;
    private String status = "No update yet";
    private Double distanceInches;
    private Double tagTxDegrees;
    private double targetVelocity;

    public VisionShooterTargeting(functions.MegaTag2Prep megaTag2, ShooterData shooterData, int goalTagId) {
        this.megaTag2 = megaTag2;
        this.shooterData = shooterData;
        this.goalTagId = goalTagId;
    }

    public void update() {
        targetAvailable = false;
        distanceInches = null;
        tagTxDegrees = null;
        targetVelocity = 0.0;

        if (megaTag2 == null) {
            status = "MegaTag2 not configured";
            return;
        }

        status = megaTag2.getGoalTagStatus(goalTagId);
        distanceInches = megaTag2.getGoalTagHorizontalDistanceInches(goalTagId);
        tagTxDegrees = megaTag2.getGoalTagTxDegrees(goalTagId);

        if (distanceInches == null || tagTxDegrees == null) {
            return;
        }

        if (shooterData == null || shooterData.isEmpty()) {
            status = "ShooterData is empty";
            return;
        }

        targetVelocity = shooterData.getTargetVelocity(distanceInches);
        targetAvailable = true;
    }

    public boolean isTargetAvailable() {
        return targetAvailable;
    }

    public String getStatus() {
        return status;
    }

    public Double getDistanceInches() {
        return distanceInches;
    }

    public Double getTagTxDegrees() {
        return tagTxDegrees;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }
}
