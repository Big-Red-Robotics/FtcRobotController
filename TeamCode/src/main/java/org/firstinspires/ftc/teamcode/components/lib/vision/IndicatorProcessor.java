package org.firstinspires.ftc.teamcode.components.lib.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public abstract class IndicatorProcessor implements VisionProcessor {
    Telemetry telemetry;
    public IndicatorProcessor(){}
    Mat mat = new Mat();
    public Rect pixel = new Rect();
    //    final int screenWidth = RobotConfig.cameraWidth;
    final int screenWidth = 640;

    private int indicator;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (frame.empty()) return frame;

        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2GRAY);
        Scalar lowHSV = new Scalar(200);
        Scalar highHSV = new Scalar(255);
        Mat thresh = new Mat();
        Mat edges = new Mat();

        //white
        Core.inRange(mat, lowHSV, highHSV, thresh);
        Imgproc.Canny(thresh, edges, 100, 200);

        List<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(edges, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        contours.removeIf(c -> Imgproc.boundingRect(c).height > 50);
        Imgproc.drawContours(frame, contours, -1, new Scalar(0, 255, 0));

        if(!contours.isEmpty()) {
            MatOfPoint pc = Collections.max(contours, Comparator.comparingDouble(Imgproc::contourArea));
            pixel = Imgproc.boundingRect(pc);

            Imgproc.rectangle(frame, new Point(pixel.x, pixel.y), new Point(pixel.x + pixel.width, pixel.y + pixel.height), new Scalar(0, 255, 0), 2);
            Imgproc.putText(frame, "Pixel " + (pixel.x + (pixel.width/2.0)) +","+(pixel.y + (pixel.height/2.0)), new Point(pixel.x, pixel.y < 10 ? (pixel.y+pixel.height+20) : (pixel.y + 20)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, new Scalar(255, 255, 255), 1);
        }

        //left: 1, middle: 2, right: 3
        if(pixel.empty()) indicator = 3;
        else if(pixel.x < (screenWidth/2)){
            indicator = 1;
        } else {
            indicator = 2;
        }

        contours.clear();
        mat.release();
        edges.release();
        thresh.release();

        return null;
    }

    public int getIndicator() {
        return indicator;
    }

    //I'm not sure if we will use this or not.
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
}