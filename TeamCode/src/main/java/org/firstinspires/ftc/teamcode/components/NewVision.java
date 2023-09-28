package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.components.lib.vision.SampleProcessor;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class NewVision {
    private VisionPortal visionPortal;

    //List of Processors that will be used
    AprilTagProcessor aprilTag;
    SampleProcessor sampleProcessor;

//    public enum Processor {
//        APRIL_TAG,
//        SAMPLE_PROCESSOR
//    }
//
//    public Processor currentProcessor;

    public NewVision(HardwareMap hardwareMap){
        //Setup vision portal
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, RobotConfig.cameraName));
        builder.setCameraResolution(new Size(RobotConfig.cameraSize[0], RobotConfig.cameraSize[1]));
        builder.enableLiveView(true);
        builder.setAutoStopLiveView(false);
            // Set the stream format; MJPEG uses less bandwidth than default YUY2.
            //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        buildAprilTagProcessor();
        builder.addProcessor(aprilTag);
        builder.addProcessor(sampleProcessor);

        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(sampleProcessor, true);
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

    void getAprilTags() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        int numberDetected = currentDetections.size();

        for (AprilTagDetection detection : currentDetections) {
            int id = detection.id;

            AprilTagPoseFtc pose = detection.ftcPose;
        }
    }
}