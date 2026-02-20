package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp
public class FlyWheelTuner extends OpMode {

    double F = 0;
    double P = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.001};
    int stepIndex = 1;
    private DcMotorEx rightFlyWheel;
    private DcMotorEx leftFlyWheel;
    double activevelocity = 6000;
    double idlevelocity = 3000;

    double curvelocity = 6000;

    public void init() {
        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "lflywheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rflywheel");
        // flywheel directions
        rightFlyWheel.setDirection(DcMotorEx.Direction.FORWARD);
        leftFlyWheel.setDirection(DcMotorEx.Direction.REVERSE);
        // flywheel encoder setup
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // flywheel PIDF setup
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        leftFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        leftFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void loop(){
        int action = 0;
        if (gamepad1.yWasPressed()) action = 1;
        if (gamepad1.bWasPressed()) action = 2;
        else if (gamepad1.dpadLeftWasPressed()) action = 3;
        else if (gamepad1.dpadRightWasPressed()) action = 4;
        else if (gamepad1.dpadUpWasPressed()) action = 5;
        else if (gamepad1.dpadDownWasPressed()) action = 6;

        switch (action) {
            case 1:
                if (curvelocity == activevelocity) curvelocity = idlevelocity;
                else curvelocity = activevelocity;
            case 2:
                stepIndex = (stepIndex + 1) % stepSizes.length;
                break;
            case 3:
                F -= stepSizes[stepIndex];
                break;
            case 4:
                F += stepSizes[stepIndex];
                break;
            case 5:
                P += stepSizes[stepIndex];
                break;
            case 6:
                P -= stepSizes[stepIndex];
                break;
        }
        double leftFlyWheelCurrent = leftFlyWheel.getCurrent(CurrentUnit.AMPS);
        double rightFlyWheelCurrent = rightFlyWheel.getCurrent(CurrentUnit.AMPS);
        double totalCurrent = leftFlyWheelCurrent + rightFlyWheelCurrent;

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        leftFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        leftFlyWheel.setVelocity(curvelocity);
        rightFlyWheel.setVelocity(curvelocity);

        telemetry.addData("Total Current", totalCurrent);
        telemetry.addData("Current Target Velocity", "%.2f", curvelocity);
        telemetry.addData("Stepsize", "%.3f", stepSizes[stepIndex]);
        telemetry.addData("P", P);
        telemetry.addData("F", F);
        telemetry.addData("Left Wheel Velocity", "%.2f", leftFlyWheel.getVelocity());
        telemetry.addData("Right Wheel Velocity", "%.2f", rightFlyWheel.getVelocity());
        telemetry.update();
    }
}
