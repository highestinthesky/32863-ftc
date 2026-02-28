package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "FlyWheelDistanceTuner", group = "Tuners")
public class FlyWheelDistanceTuner extends OpMode {
    private static final double P = 3.0;
    private static final double I = 0.0;
    private static final double D = 0.0;
    private static final double F = 7.0;
    private static final double P2 = 4.0;
    private static final double I2 = 0.0;
    private static final double D2 = 0.0;
    private static final double F2 = 5.0;
    double[] stepSizes = {1000.0, 500.0, 100.0, 10.0, 1.0};
    int stepIndex = 2;
    private DcMotorEx rightFlyWheel;
    private DcMotorEx leftFlyWheel;
    private DcMotorEx rightintake;
    private DcMotorEx leftintake;
    double curvelocity = 0;
    private Servo turrethood;
    double turrethoodvalue;

    public void init() {
        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "lflywheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rflywheel");

        // temp intake setups
        rightintake = hardwareMap.get(DcMotorEx.class, "rightintake");
        leftintake = hardwareMap.get(DcMotorEx.class, "leftintake");

        rightintake.setDirection(DcMotorEx.Direction.FORWARD);
        leftintake.setDirection(DcMotorEx.Direction.REVERSE);
        rightintake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftintake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients intakePidf = new PIDFCoefficients(P2, I2, D2, F2);
        rightintake.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, intakePidf);
        leftintake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, intakePidf);
        rightintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // flywheel directions
        rightFlyWheel.setDirection(DcMotorEx.Direction.REVERSE);
        leftFlyWheel.setDirection(DcMotorEx.Direction.FORWARD);
        // flywheel encoder setup
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // flywheel PIDF setup
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        leftFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        leftFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turrethood = hardwareMap.get(Servo.class, "tservo");
        turrethood.setDirection(Servo.Direction.REVERSE);
    }

    public void loop() {
        int action = 0;
        if (gamepad1.bWasPressed()) action = 1;
        else if (gamepad1.dpadLeftWasPressed()) action = 2;
        else if (gamepad1.dpadRightWasPressed()) action = 3;
        else if (gamepad1.yWasPressed()) action = 6;
        else if (gamepad1.aWasPressed()) action = 7;
        switch (action) {
            case 1:
                stepIndex = (stepIndex + 1) % stepSizes.length;
                break;
            case 2:
                curvelocity = Math.max(0, curvelocity - stepSizes[stepIndex]);
                break;
            case 3:
                curvelocity += stepSizes[stepIndex];
                break;
            case 6:
                if (turrethoodvalue < 1) turrethoodvalue += 0.1;
                break;
            case 7:
                if (turrethoodvalue > 0) turrethoodvalue -= 0.1;
                break;
        }

        if (gamepad1.right_trigger > 0.4){
            rightintake.setVelocity(6000);
        } else{
            rightintake.setVelocity(0);
        }

        if (gamepad1.left_trigger > 0.4){
            leftintake.setVelocity(6000);
        } else {
            leftintake.setVelocity(0);
        }



        turrethood.setPosition(turrethoodvalue);

        double leftFlyWheelCurrent = leftFlyWheel.getCurrent(CurrentUnit.AMPS);
        double rightFlyWheelCurrent = rightFlyWheel.getCurrent(CurrentUnit.AMPS);
        double totalCurrent = leftFlyWheelCurrent + rightFlyWheelCurrent;

        leftFlyWheel.setVelocity(curvelocity);
        rightFlyWheel.setVelocity(curvelocity);

        telemetry.addData("Total Current (Amps)", totalCurrent);
        telemetry.addData("Stepsize", "%.3f", stepSizes[stepIndex]);
        telemetry.addData("hood", turrethoodvalue);
        telemetry.addData("Target Velocity", curvelocity);
        telemetry.addData("PIDF", "P=%.1f I=%.1f D=%.1f F=%.1f", P, I, D, F);
        telemetry.addData("Left Wheel Velocity", "%.2f", leftFlyWheel.getVelocity());
        telemetry.addData("Right Wheel Velocity", "%.2f", rightFlyWheel.getVelocity());
        telemetry.update();
    }
}
