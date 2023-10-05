package org.firstinspires.ftc.teamcode.components.old;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.components.lib.vision.old.AutonDetector;
import org.firstinspires.ftc.teamcode.components.lib.vision.old.SleeveDetector;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class Vision {
    private DistanceSensor distanceSensor;
    private final OpenCvCamera camera;

    public Vision(HardwareMap hardwareMap){
        this.distanceSensor = hardwareMap.get(DistanceSensor.class, RobotConfig.distanceSensor);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName cameraName = hardwareMap.get(WebcamName.class, RobotConfig.cameraName);
        this.camera = OpenCvCameraFactory.getInstance().createWebcam(cameraName, cameraMonitorViewId);

        sleeveDetector = new SleeveDetector();
        autonDetector = new AutonDetector();
        setDetector("sleeve");
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(RobotConfig.cameraWidth, RobotConfig.cameraHeight, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    SleeveDetector sleeveDetector;
    AutonDetector autonDetector;
    AprilTagDetection tagOfInterest = null;

    private String currentDetector = "";
    public void setDetector(String d) {
        if(d.equals("sleeve") && !currentDetector.equals("sleeve")) {
            camera.setPipeline(sleeveDetector);
        } else {
            if(d.equals("pole")) autonDetector.detectMode = AutonDetector.DetectMode.POLE;
            else if(d.equals("cone")) autonDetector.detectMode = AutonDetector.DetectMode.CONE;
            if(!currentDetector.equals("pole") && !currentDetector.equals("cone")) camera.setPipeline(autonDetector);
        }
        currentDetector = d;
    }

    public AutonDetector getAutonPipeline(){
        return autonDetector;
    }

    public void searchTags() {
        ArrayList<AprilTagDetection> currentDetections = sleeveDetector.getLatestDetections();

        for(AprilTagDetection tag : currentDetections) {
            if(tag.id == 1 || tag.id == 2 || tag.id == 3) {
                tagOfInterest = tag;
                break;
            }
        }
    }

    public int tagId() {
        if(tagOfInterest != null) return tagOfInterest.id;
        else return -1;
    }

    public double distance() {
        return distanceSensor.getDistance(DistanceUnit.MM);
    }
}
