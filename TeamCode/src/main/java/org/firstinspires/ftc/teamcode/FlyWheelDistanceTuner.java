package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class FlyWheelDistanceTuner extends OpMode {
    double state1 = 1.8;
    double state2 = 0.5;
    double F = 0.5;
    double P = 0.5;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.001};
    int stepIndex = 1;
    private DcMotorEx rightFlyWheel;
    private DcMotorEx leftFlyWheel;
    double curvelocity = 6000;
    boolean high = false;
    private Servo turrethood;
    double turrethoodvalue;

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

        turrethood = hardwareMap.get(Servo.class, "tservo");
        turrethood.setDirection(Servo.Direction.REVERSE);
    }

    public void loop(){
        int action = 0;
        if (gamepad1.bWasPressed()) action = 1;
        else if (gamepad1.dpadLeftWasPressed()) action = 2;
        else if (gamepad1.dpadRightWasPressed()) action = 3;
        else if (gamepad1.dpadUpWasPressed()) action = 4;
        else if (gamepad1.dpadDownWasPressed()) action = 5;
        else if (gamepad1.yWasPressed()) action = 6;
        else if (gamepad1.aWasPressed()) action = 7;
        switch (action) {
            case 1:
                stepIndex = (stepIndex + 1) % stepSizes.length;
                break;
            case 2:
                F -= stepSizes[stepIndex];
                break;
            case 3:
                F += stepSizes[stepIndex];
                break;
            case 4:
                P += stepSizes[stepIndex];
                break;
            case 5:
                P -= stepSizes[stepIndex];
                break;
            case 6:
                if (turrethoodvalue < 1) turrethoodvalue += 0.1;
                break;
            case 7:
                if (turrethoodvalue > 0) turrethoodvalue -= 0.1;
                break;
        }
        turrethood.setPosition(turrethoodvalue);


        double leftFlyWheelCurrent = leftFlyWheel.getCurrent(CurrentUnit.AMPS);
        double rightFlyWheelCurrent = rightFlyWheel.getCurrent(CurrentUnit.AMPS);
        double totalCurrent = leftFlyWheelCurrent + rightFlyWheelCurrent;

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        leftFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        leftFlyWheel.setVelocity(curvelocity);
        rightFlyWheel.setVelocity(curvelocity);

        telemetry.addData("Total Current (Amps)", totalCurrent);
        telemetry.addData("Stepsize", "%.3f", stepSizes[stepIndex]);
        telemetry.addData("hood", turrethoodvalue);
        telemetry.addData("P", P);
        telemetry.addData("F", F);
        telemetry.addData("Left Wheel Velocity", "%.2f", leftFlyWheel.getVelocity());
        telemetry.addData("Right Wheel Velocity", "%.2f", rightFlyWheel.getVelocity());
        telemetry.update();
    }
}
