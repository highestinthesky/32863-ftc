package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "DcMotorTuner", group = "Tuners")
public class DcMotorTuner extends OpMode {
    public static String MOTOR_NAME = "frontLeftMotor";
    public static boolean REVERSE = false;
    public static boolean BRAKE_AT_ZERO = false;

    private DcMotor motor;
    private String initError;

    public static DcMotor bindMotor(HardwareMap hardwareMap, String motorName) {
        return hardwareMap.get(DcMotor.class, motorName);
    }

    public static void configureMotor(DcMotor motor, boolean brakeAtZero) {
        if (motor == null) return;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(brakeAtZero
                ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public static void runMax(DcMotor motor, boolean reverse) {
        if (motor == null) return;
        motor.setPower(reverse ? -1.0 : 1.0);
    }

    public static void stopMotor(DcMotor motor) {
        if (motor == null) return;
        motor.setPower(0.0);
    }

    @Override
    public void init() {
        try {
            motor = bindMotor(hardwareMap, MOTOR_NAME);
            configureMotor(motor, BRAKE_AT_ZERO);
            initError = null;
        } catch (Exception e) {
            motor = null;
            initError = e.getClass().getSimpleName() + ": " + e.getMessage();
        }
        printTelemetry(telemetry, MOTOR_NAME, motor, initError, REVERSE);
    }

    @Override
    public void loop() {
        if (motor != null) {
            runMax(motor, REVERSE);
        }
        printTelemetry(telemetry, MOTOR_NAME, motor, initError, REVERSE);
    }

    @Override
    public void stop() {
        stopMotor(motor);
    }

    public static void printTelemetry(Telemetry telemetry, String motorName, DcMotor motor, String error, boolean reverse) {
        telemetry.addLine("DC Motor Tuner");
        telemetry.addData("Motor Name", motorName);
        telemetry.addData("Reverse", reverse);
        telemetry.addData("Target Power", reverse ? -1.0 : 1.0);
        if (error != null) {
            telemetry.addData("Status", "ERROR");
            telemetry.addData("Reason", error);
        } else {
            telemetry.addData("Status", motor == null ? "No motor" : "Running max");
            if (motor != null) telemetry.addData("Encoder", motor.getCurrentPosition());
        }
        telemetry.update();
    }
}
