package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.teaminfo.InitialSide;
import org.firstinspires.ftc.teamcode.utility.teaminfo.TeamColor;

@Config
public class RobotConfig {
    public static TeamColor teamColor = TeamColor.BLUE;
    public static InitialSide initialSide = InitialSide.RIGHT;
    public static boolean isRight = RobotConfig.initialSide == InitialSide.RIGHT;
    public static boolean isRed = RobotConfig.teamColor == TeamColor.RED;

    public static String motorFL = "dFL", motorFR = "dFR", motorBL = "dBL", motorBR = "dBR";
    public static String liftR = "liftR", liftL = "liftL";
    public static String clawRotator = "wrist";
    public static String clawR = "clawR", clawL = "clawL";
    public static String launchDrone = "droneLauncher";
    public static String positionDrone = "dronePositioner";

    //unused
    public static String imu = "imu";
    public static String distanceSensor = "distanceSensor";

    //Webcam: Logitech HD Webcam C270
    public static String cameraName = "Webcam 1";
    public static int cameraWidth = 640;
    public static int cameraHeight = 480;
}
