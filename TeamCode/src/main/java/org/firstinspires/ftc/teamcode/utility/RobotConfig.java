package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.teaminfo.InitialSide;
import org.firstinspires.ftc.teamcode.utility.teaminfo.TeamColor;

@Config
public class RobotConfig {
    public static TeamColor teamColor = TeamColor.BLUE;
    public static InitialSide initialSide = InitialSide.RIGHT;

    public static String [] motors = {"motorFL", "motorFR", "motorBL", "motorBR"};
    public static String [] lifts = {"leftArm", "rightArm"};
    public static String clawRotator = "gripper";
    public static String launchDrone = "droneLauncher";
    public static String positionDrone = "dronePositioner";
    public static String claw = "cam";
    public static String imu = "imu";
    public static String distanceSensor = "distanceSensor";

    //Webcam: Logitech HD Webcam C270
    public static String cameraName = "Webcam 1";
    public static int cameraWidth = 640;
    public static int cameraHeight = 480;
}
