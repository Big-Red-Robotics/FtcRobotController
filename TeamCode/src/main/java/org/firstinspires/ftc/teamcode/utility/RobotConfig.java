package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.teaminfo.InitialSide;
import org.firstinspires.ftc.teamcode.utility.teaminfo.TeamColor;
//u only change the bounded buttons in the BaseTeleOp.java file.
@Config
public class RobotConfig {
    public static TeamColor teamColor = TeamColor.BLUE;
    public static InitialSide initialSide = InitialSide.LEFT;

    // ------- Other configs.

    public static double joyStickSpeed = 0.7;

    //-------- Components

    public static final String intakeName = "intake";

    //newArm refers to the arm that spins the intake.
    public static final String NewArm = "newArm";

    public static final String motorFL = "dFL", motorFR = "dFR", motorBL = "dBL", motorBR = "dBR";

    //unused
   public static final String imu = "imu";

   //----------- :ift

   public static final String lift = "lift";

    // here are the configurations of the lift which requires testing for the exact values. Given my experience, sometimes the encoder get position can be a little bit buggy so please be careful while testing the code. It will be beneficial if you can slow the setPower.
   // TODO get the correct value for the encoder values;
    public static final int liftMaxEncoder = 100000;

    // TODO i am not sure but the initial value can be either zero or we can just use the current value and set it as the lowest point assuming it is the minium.

    public static final int liftMinEncoder = 0;

    //TODO if resetting encoders provides a different value everytime, we can add the current position with the liftDifference to get the lift maxium.
    public static final int liftDifference = 4560;

    // This is the stop buffer to prevent it exceeding the limit given its momentum.
    public static final int liftBuffer = 20;

    //----------- Claw

    public static final int clawCloseAndOpenValue = 30;

    public static final String claw = "claw";




}
