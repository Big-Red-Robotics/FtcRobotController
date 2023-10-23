package org.firstinspires.ftc.teamcode.components.lib.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    //Look at teamcode/components/lib/vision/TestProcessor.java for a similar program.

    Telemetry telemetry;
    public IndicatorProcessor(){}
    Mat mat = new Mat();
    public Rect pixel = new Rect();
    final int screenWidth = 640;

    private int indicator;

    //use these if you want to...
//    final int cameraWidth = RobotConfig.cameraWidth;
//    final int cameraHeight = RobotConfig.cameraHeight;

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        //frame is the input image you will get. Ignore captureTimeNanos for now
        if (frame.empty()) return frame;

        //TODO: Detect white object and take the x-coordinate
        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(0,0,168);
        Scalar highHSV = new Scalar(172, 111, 255);
        Mat thresh = new Mat();
        Mat edges = new Mat();

        //white
        Core.inRange(mat, lowHSV, highHSV, thresh);
        Imgproc.Canny(thresh, edges, 100, 200);

        List<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(edges, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//        contours.removeIf(c -> Imgproc.boundingRect(c).height < 20);
        Imgproc.drawContours(frame, contours, -1, new Scalar(255, 255, 255));

        if(!contours.isEmpty()) {// identifies the largest contour and draws stuff around it + coordinates
//            MatOfPoint biggestPole = Collections.max(contours, Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).width));
//            pixel = Imgproc.boundingRect(biggestPole);
            MatOfPoint pc = Collections.max(contours, Comparator.comparingDouble(Imgproc::contourArea));
            pixel = Imgproc.boundingRect(pc);

            Imgproc.rectangle(frame, new Point(pixel.x, pixel.y), new Point(pixel.x + pixel.width, pixel.y + pixel.height), new Scalar(0, 255, 0), 2);
//            Imgproc.rectangle(frame, pixel, new Scalar(255, 0, 0), 2);
//            Imgproc.circle(frame, new Point(pixel.x + (pixel.width/2), pixel.y + (pixel.height/2)), 1, new Scalar(255, 0, 255), 3);
            Imgproc.putText(frame, "Pixel " + (pixel.x + (pixel.width/2.0)) +","+(pixel.y + (pixel.height/2.0)), new Point(pixel.x, pixel.y < 10 ? (pixel.y+pixel.height+20) : (pixel.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255, 255, 255), 2);
        }

        //TODO: Update the indicator value.
        int location = 0;
        //Divide the screen vertically into 3...
        //Determine which subdivided screen the white object is at...
        //left: 1, middle: 2, right: 3
        int w1 = screenWidth/3;
        int w2 = 2*(screenWidth/3);
        int w3 = screenWidth;
        if(pixel.x>=0 && pixel.x<=w1){
            location = 1;
        } else if (pixel.x>w1 && pixel.x<=w2) {
            location = 2;
        } else if (pixel.x>w2 && pixel.x<=w3) {
            location = 3;
        }
        indicator = location;

        //TODO: Make sure to .release() all mats to prevent memory leak
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
