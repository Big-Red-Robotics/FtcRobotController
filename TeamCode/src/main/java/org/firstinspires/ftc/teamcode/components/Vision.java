package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.components.lib.vision.IndicatorProcessor;
import org.firstinspires.ftc.teamcode.components.lib.vision.TestProcessor;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Vision {
    private VisionPortal visionPortal;
    private final int screenWidth = RobotConfig.cameraWidth;

    //List of Processors that will be used
    AprilTagProcessor aprilTag;
    TestProcessor testProcessor;
    public IndicatorProcessor indicatorProcessor = new IndicatorProcessor();

    VisionProcessor[] processors = {aprilTag, testProcessor, indicatorProcessor};

//    public enum Processor {
//        APRIL_TAG,
//        SAMPLE_PROCESSOR
//    }
//
//    public Processor currentProcessor;

    public Vision(HardwareMap hardwareMap){
        //Setup vision portal
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, RobotConfig.cameraName));
        builder.setCameraResolution(new Size(RobotConfig.cameraWidth, RobotConfig.cameraHeight));
        builder.enableLiveView(true);
        builder.setAutoStopLiveView(false);
            // Set the stream format; MJPEG uses less bandwidth than default YUY2.
            //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        //add processors to the builder
        buildAprilTagProcessor();

        //add built processors to VisionPortal
        builder.addProcessor(indicatorProcessor);

        visionPortal = builder.build();

        // Disable or re-enable the processor
//        visionPortal.setProcessorEnabled(indicatorProcessor, true);
    }

    void buildAprilTagProcessor() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
//                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                .build();

        //close or open streaming
//        visionPortal.stopStreaming();

        //close portal
//        visionPortal.close();
    }

    public void setProcessor(VisionProcessor processor) {
        visionPortal.setProcessorEnabled(processor, true);
    }

    public int getIndicator() {
        //left: 1, right: 2, no indicator: 3
        if(indicatorProcessor.getIndicator().empty()) return 3;
        else if(indicatorProcessor.getIndicator().x > (screenWidth/2)) return 2;
        else return 1;
    }

    void getAprilTags() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        int numberDetected = currentDetections.size();

        for (AprilTagDetection detection : currentDetections) {
            int id = detection.id;

            AprilTagPoseFtc pose = detection.ftcPose;
        }
    }
}