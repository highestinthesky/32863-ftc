package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IPController
{
    private final Telemetry telemetry;
    private final double P;
    private final double I;
    private final double minErrorChange;
    private final double maxOutput;

    private double targetPos;
    private double error;
    private double errorPrevious;
    private double errorSum;
    private double errorChange;

    public IPController(double targetPos, double P, double I, double minErrorChange, double maxOutput, Telemetry telemetry) {
        this.targetPos = targetPos;
        this.P = P;
        this.I = I;
        this.minErrorChange = minErrorChange;
        this.maxOutput = maxOutput;
        this.telemetry = telemetry;
        errorSum = 0.0;
        errorPrevious = 0.0;
        error = 0.0;
    }

    public void setTargetPos( double targetPos ) {
        this.targetPos = targetPos;
        errorSum = 0.0;
        errorPrevious = 0.0;
        error = 0.0;
    }

    public void adjustTargetPos( double adj)
    {
        this.targetPos += adj;
    }

    public double getTargetPos() {
        return this.targetPos;
    }

    public double getPower(double currentPos)
    {
        error = targetPos - currentPos;
        if (Math.abs(error - errorPrevious) < minErrorChange ) {
            errorSum += error;
        } else {
            errorSum = 0;
        }
        errorChange = error- errorPrevious;
        errorPrevious = error;
        return Range.clip((error * P + errorSum * I), -maxOutput, maxOutput);
    }

    public double getError() {
        return error;
    }

    public double getErrorChange() {
        return errorChange;
    }
}