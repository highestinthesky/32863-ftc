package org.firstinspires.ftc.teamcode.vision;

import static org.opencv.core.Core.inRange;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


public class MarkerVisionProcessor implements VisionProcessor {

    private Location location = Location.NONE;
    public Rect rectMiddle = new Rect(100, 120, 250, 180);
    public Rect rectRight = new Rect(500, 165, 100, 200);
    Mat submat = new Mat();
    Mat hsvMat = new Mat();
    Mat hsvFilteredMat = new Mat();

    public Location getLocation() {
        return location;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
//        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        Mat outframe = new Mat();
        inRange(frame, new Scalar(20.0,20.0,20.0,0.0), new Scalar(255.0,70.0,255.0,255.0), outframe);

        Mat m = new Mat(outframe, rectMiddle);
        int middleCount = Core.countNonZero(m);

        Mat r = new Mat(outframe, rectRight);
        int rightCount = Core.countNonZero(r);
        String middle = "" + middleCount;
        String right = "" + rightCount;
        Point mpos = new Point(90, 110);
        Point rpos = new Point(500, 155);
        int fontFace = Imgproc.FONT_HERSHEY_SIMPLEX;
        Scalar color = new Scalar(255, 0, 0);
        Imgproc.putText(outframe, middle, mpos, fontFace, 1,  color);
        Imgproc.putText(outframe, right, rpos, fontFace, 1,  color);

        if(middleCount > 1000){
            location = Location.MIDDLE;
        } else if (rightCount > 1000) {
            location = Location.RIGHT;
        }else {
            location = Location.LEFT;
        }
        return location;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }


    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    public enum Location {
        NONE,
        LEFT,
        MIDDLE,
        RIGHT
    }
}