package org.firstinspires.ftc.teamcode.components.lib;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.teaminfo.TeamColor;

public class Positions {
    //config
    boolean isRight, isRed, isCloseToBackdrop;

    public static Pose2d startPose, prePixel, dropPixel, backDrop, park;

    //indicator
    int indicator;
    int LEFT = 1, MIDDLE = 2, RIGHT = 3;

    public Positions(boolean isRight, boolean isRed, int indicator){
        this.isRight = isRight;
        this.isRed = isRed;
        this.indicator = indicator;

        this.isCloseToBackdrop = isRight == isRed;
    }

    void updatePositions(){
        int color = (isRed) ? 1 : -1;
        int initialHeading = (isRed) ? 180 : 0;

        startPose = new Pose2d(65 * color, (isCloseToBackdrop) ? 10 : -34, Math.toRadians(initialHeading));

        if(isCloseToBackdrop){
            park = new Pose2d(62 * color, 45, Math.toRadians(-90));
            prePixel = new Pose2d(36 * color, 24, Math.toRadians(-90));
            if(indicator == MIDDLE){
                dropPixel = new Pose2d(25 * color, 24, Math.toRadians(-90));
                if(RobotConfig.teamColor == TeamColor.BLUE) backDrop = new Pose2d(-42, 54, Math.toRadians(90));
                else backDrop = new Pose2d(35, 48, Math.toRadians(90));
            } else if((indicator == LEFT) == (RobotConfig.teamColor == TeamColor.BLUE)){
                //left for blue, right for red (furthest from the center)
                dropPixel = new Pose2d(32 * color, 35, Math.toRadians(-90));
                if(RobotConfig.teamColor == TeamColor.BLUE) backDrop = new Pose2d(-53, 54, Math.toRadians(90));
                else backDrop = new Pose2d(43, 48, Math.toRadians(90));
            } else {
                //right for blue, left for red (closest to the center)
                dropPixel = new Pose2d(32 * color, 13, Math.toRadians(-90));
                if(RobotConfig.teamColor == TeamColor.BLUE) backDrop = new Pose2d(-37, 54, Math.toRadians(90));
                else backDrop = new Pose2d(27, 48, Math.toRadians(90));
            }
        } else {
            prePixel = new Pose2d(43 * color, -34, Math.toRadians(initialHeading));
            park = startPose;
            if(indicator == MIDDLE) dropPixel = new Pose2d(38 * color, -33, Math.toRadians(initialHeading));
            else if((indicator == RIGHT) == (RobotConfig.teamColor == TeamColor.BLUE)) dropPixel = new Pose2d(38 * color, -56, Math.toRadians(initialHeading - (45*color)));
            else dropPixel = new Pose2d(38 * color, -33, Math.toRadians(initialHeading - (45*color)));
            backDrop = dropPixel; //is not used (cannot leave null for trajectory building)
        }
    }
}
