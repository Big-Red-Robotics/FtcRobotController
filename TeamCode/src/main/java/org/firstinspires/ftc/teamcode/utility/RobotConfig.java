package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.teaminfo.InitialSide;
import org.firstinspires.ftc.teamcode.utility.teaminfo.TeamColor;

@Config
public class RobotConfig {
    public static TeamColor teamColor = TeamColor.BLUE;
    public static InitialSide initialSide = InitialSide.LEFT;

    public static String motorFL = "dFL", motorFR = "dFR", motorBL = "dBL", motorBR = "dBR";
    public static String liftR = "liftR", liftL = "liftL";
    public static String armExtension = "armEx";
    public static String clawRotator = "wrist";
    public static String leftClaw = "leftClaw";
    public static String rightClaw = "rightClaw";
    public static String launchDrone = "droneLauncher";
    public static String positionDrone = "dronePositioner";
    public static String clawPivot = "wrist2";

    //unused
    public static String imu = "imu";
    public static String distanceSensor = "distance";

    //Webcam: Logitech HD Webcam C270
    public static String cameraName = "Webcam 1";
    public static int cameraWidth = 640;
    public static int cameraHeight = 480;

    //blinkin
    public static String blinkinLeft = "blinkinL";
    public static String blinkinRight = "blinkinR";
}
