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
    public static String gripper = "gripper";
    public static String cam = "cam";
    public static String imu = "imu";
    public static String camera = "Webcam 1";
    public static String distanceSensor = "distanceSensor";
    public static int [] cameraSize = {640, 480};
}
