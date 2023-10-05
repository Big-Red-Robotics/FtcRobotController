package org.firstinspires.ftc.teamcode.components.lib.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

public abstract class IndicatorProcessor implements VisionProcessor {
    //Look at teamcode/components/lib/vision/TestProcessor.java for a similar program.

    private int indicator;

    //use these if you want to...
    final int cameraWidth = RobotConfig.cameraWidth;
    final int cameraHeight = RobotConfig.cameraHeight;

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        //frame is the input image you will get. Ignore captureTimeNanos for now
        if (frame.empty()) return frame;

        //TODO: Detect white object and take the x-coordinate

        //TODO: Update the indicator value.
            //Divide the screen vertically into 3...
            //Determine which subdivided screen the white object is at...
            //left: 1, middle: 2, right: 3

        //TODO: Make sure to .release() all mats to prevent memory leak
        return null;
    }

    public int getIndicator() {
        return indicator;
    }

    //I'm not sure if we will use this or not.
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
}
