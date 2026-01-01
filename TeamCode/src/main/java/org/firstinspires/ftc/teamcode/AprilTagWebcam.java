package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import android.util.Size;

@SuppressWarnings("unused")
public class AprilTagWebcam {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedId) {
        if (detectedId == null) {
            return;
        }
        if (detectedId.metadata != null) {
            telemetry.addLine(String.format(Locale.US,
                    "\n==== (ID %d) %s",
                    detectedId.id,
                    detectedId.metadata.name));

            // Changed (inch) to (cm) to match .setOutputUnits(DistanceUnit.CM, ...)
            telemetry.addLine(String.format(Locale.US,
                    "XYZ %6.1f %6.1f %6.1f  (cm)",
                    detectedId.ftcPose.x,
                    detectedId.ftcPose.y,
                    detectedId.ftcPose.z));

            telemetry.addLine(String.format(Locale.US,
                    "PRY %6.1f %6.1f %6.1f  (deg)",
                    detectedId.ftcPose.pitch,
                    detectedId.ftcPose.roll,
                    detectedId.ftcPose.yaw));

            telemetry.addLine(String.format(Locale.US,
                    "RBE %6.1f %6.1f %6.1f  (cm, deg, deg)",
                    detectedId.ftcPose.range,
                    detectedId.ftcPose.bearing,
                    detectedId.ftcPose.elevation));
            telemetry.update();

        } else {
            telemetry.addLine(String.format(Locale.US,
                    "\n==== (ID %d) Unknown",
                    detectedId.id));

            telemetry.addLine(String.format(Locale.US,
                    "Center %6.0f %6.0f (pixels)",
                    detectedId.center.x,
                    detectedId.center.y));
            telemetry.update();
        }

    }

    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
