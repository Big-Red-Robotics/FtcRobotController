package org.firstinspires.ftc.teamcode.components.lib.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.teaminfo.TeamColor;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class IndicatorProcessor implements VisionProcessor {
    Mat mat = new Mat();
    public Rect pixel = new Rect();

    public IndicatorProcessor(){}

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (frame.empty()) return frame;

        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);
        Mat thresh = new Mat();
        if (RobotConfig.teamColor == TeamColor.RED) thresh = redThresh();
        else if (RobotConfig.teamColor == TeamColor.BLUE) thresh = blueThresh();
//        thresh = blueThresh();

        Imgproc.erode(thresh, thresh, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

        List<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(thresh, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//        contours.removeIf(c -> Imgproc.boundingRect(c).height > 50 || Imgproc.boundingRect(c).area() < 200);
        Imgproc.drawContours(frame, contours, -1, new Scalar(0, 255, 0));

        pixel = new Rect();
        if(!contours.isEmpty()) {
            MatOfPoint pc = Collections.max(contours, Comparator.comparingDouble(Imgproc::contourArea));
            pixel = Imgproc.boundingRect(pc);

            Imgproc.rectangle(frame, new Point(pixel.x, pixel.y), new Point(pixel.x + pixel.width, pixel.y + pixel.height), new Scalar(0, 255, 0), 2);
            Imgproc.putText(frame, "Pixel " + (pixel.x + (pixel.width/2.0)) +","+(pixel.y + (pixel.height/2.0)), new Point(pixel.x, pixel.y < 10 ? (pixel.y+pixel.height+20) : (pixel.y + 20)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
        }

        contours.clear();
        mat.release();
        thresh.release();
      
        return null;
    }

    private Mat redThresh() {
        Scalar red1_lowHSV = new Scalar (0, 100, 100);
        Scalar red1_highHSV = new Scalar (10, 255, 255);
        Mat thresh1 = new Mat();

        Scalar red2_lowHSV = new Scalar(160, 100, 100);
        Scalar red2_highHSV = new Scalar (180, 255, 255);
        Mat thresh2 = new Mat();

        Core.inRange(mat, red1_lowHSV, red1_highHSV, thresh1);
        Core.inRange(mat, red2_lowHSV, red2_highHSV, thresh2);
        Mat thresh = new Mat();
        Core.bitwise_or(thresh1, thresh2, thresh);

        thresh1.release();
        thresh2.release();

        return thresh;
    }

    private Mat blueThresh() {
        Scalar blue_lowHSV = new Scalar (105,10,0);
        Scalar blue_highHSV = new Scalar (130,255,255);
        Mat thresh = new Mat();
        Core.inRange(mat, blue_lowHSV, blue_highHSV, thresh);

        return thresh;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Rect getIndicator() {
        return pixel;
    }
}