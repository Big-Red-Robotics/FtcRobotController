package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Vision;
import org.firstinspires.ftc.teamcode.components.lib.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.components.lib.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.teaminfo.InitialSide;
import org.firstinspires.ftc.teamcode.utility.teaminfo.TeamColor;

@Autonomous(name="2023-24 CenterStage", preselectTeleOp = "Jan 14 TeleOp")
public class NewAutonomous extends LinearOpMode {
    //indicator
    int indicator;
    int LEFT = 1, MIDDLE = 2, RIGHT = 3;

    //team info
    boolean isRight, isRed;

    //odometry spots
    Pose2d startPose, prePixel, dropPixel, park, backDrop, intermediate;

    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Chassis chassis = new Chassis(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            indicator = vision.getIndicator();
            updateSideConfiguration();

            if (gamepad1.left_bumper || gamepad2.left_bumper) arm.setLiftPosition(arm.low);
            if (gamepad1.right_bumper || gamepad2.right_bumper) arm.setLiftPosition(arm.ground);

            arm.update(false);

            telemetry.addLine("USE BUMPER FOR ARM TEST");
            telemetry.addData("arm encoder position", arm.getLiftPosition());
            telemetry.addData("Team color", RobotConfig.teamColor);
            telemetry.addData("Initial side", RobotConfig.initialSide);
            telemetry.addData("Detected indicator", indicator);
            telemetry.update();

            sleep(100);
        }

        //put claw down (claw flipped up for initialization due to 18-inch restriction)
        arm.setClawRotatorPosition(0.007);
        waitSeconds(5); //TODO

        //update indicator information
        indicator = vision.getIndicator();
        updateCoordinates();
        telemetry.addData("Detected indicator", indicator);
        telemetry.update();

        chassis.setPoseEstimate(startPose);

        //ALL, startPosition ~ pixel location
        TrajectorySequence traj1 = chassis.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(prePixel)
                .lineToLinearHeading(dropPixel)
                .build();

        //CLOSE TO BACKDROP, dropped pixel ~ backdrop
        TrajectorySequence traj2 = chassis.trajectorySequenceBuilder(dropPixel)
                .back(5)
                .addDisplacementMarker(() -> arm.toPosition(arm.low, 1,true))
                .turn(Math.toRadians(180))
                .lineToLinearHeading(backDrop, Chassis.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Chassis.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //CLOSE TO BACKDROP, backdrop ~ park
        TrajectorySequence traj3 = chassis.trajectorySequenceBuilder(backDrop)
                .back(10,
                        Chassis.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Chassis.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(park)
                .build();

        //FAR FROM BACKDROP, dropped pixel ~ backdrop
        TrajectorySequence traj4 = chassis.trajectorySequenceBuilder(dropPixel)
                .back(5)
                .lineToLinearHeading(intermediate)
                .forward(36)
                .turn(Math.toRadians((isRed) ? -90 : 90))
                .strafeTo(new Vector2d((isRed) ? 10 : -10, 20))
                .addDisplacementMarker(() -> arm.toPosition(arm.low, 1,true))
                .lineToLinearHeading(backDrop)
                .build();

        //actual autonomous sequence
        chassis.followTrajectorySequence(traj1);
        if(RobotConfig.teamColor == TeamColor.BLUE) arm.openRightClaw();
        else arm.openLeftClaw();
        waitSeconds(0.6);
        arm.setClawRotatorPosition(0.8);
        waitSeconds(0.5);

        if(isRight == isRed)chassis.followTrajectorySequence(traj2);
        else chassis.followTrajectorySequence(traj4);

        //at backdrop
        if(RobotConfig.teamColor == TeamColor.BLUE) arm.openLeftClaw();
        else arm.openRightClaw();
        waitSeconds(0.6);
        chassis.followTrajectorySequence(traj3);
        arm.closeClaw();
        arm.toPosition(arm.low, 1,false);
        waitSeconds(1.0);
        arm.toPosition(arm.ground, 0, false);
    }

    void waitSeconds(double seconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds() < seconds);
    }

    void updateCoordinates(){
        boolean closeToBackdrop = isRight == isRed;
        int color = (isRed) ? 1 : -1;
        int initialHeading = (isRed) ? 180 : 0;
        startPose = new Pose2d(65 * color, (closeToBackdrop) ? 10 : -34, Math.toRadians(initialHeading));
        park = new Pose2d(10 * color, 47, Math.toRadians(90)); //final position

        if(closeToBackdrop){
//            park = new Pose2d(62 * color, 47, Math.toRadians(90)); //final position, corner
            prePixel = new Pose2d(36 * color, 24, Math.toRadians(-90)); //middle position before first pixel
            intermediate = park; //will not use
            if(indicator == MIDDLE){
                /*middle for both colors*/
                dropPixel = new Pose2d(25 * color, 24, Math.toRadians(-90));
            } else if((indicator == LEFT) == (RobotConfig.teamColor == TeamColor.BLUE)){
                /*left for blue, right for red (furthest from the center)*/
                dropPixel = new Pose2d(32 * color, 35, Math.toRadians(-90));
            } else {
                //right for blue, left for red (closest to the center)
                dropPixel = new Pose2d(32 * color, 13, Math.toRadians(-90));
            }
        } else {
//            park = new Pose2d(10 * color, 47, Math.toRadians(90)); //final position
            prePixel = new Pose2d(43 * color, -34, Math.toRadians(initialHeading)); //middle position before first pixel
            if(indicator == MIDDLE) {
                dropPixel = new Pose2d(38 * color, -33, Math.toRadians(initialHeading));
                intermediate = new Pose2d((isRed) ? 48 : -48, -60, Math.toRadians((isRed) ? 180 : 0));
            } else if((indicator == LEFT) == (RobotConfig.teamColor == TeamColor.BLUE)) {
                /*left for blue, right for red (furthest from the center)*/
                dropPixel = new Pose2d(38 * color, -35, Math.toRadians(initialHeading - (45*color)));
                intermediate = new Pose2d((isRed) ? 48 : -48, -46, Math.toRadians((isRed) ? 180 : 0));
            } else {
                //right for blue, left for red (closest to the center)
                dropPixel = new Pose2d(43 * color, -50, Math.toRadians(initialHeading));
                intermediate = new Pose2d((isRed) ? 48 : -48, -33, Math.toRadians((isRed) ? 180 : 0));
            }
        }

        if(indicator == MIDDLE){
            /*middle for both colors*/
            if(RobotConfig.teamColor == TeamColor.BLUE) backDrop = new Pose2d(-42, 54, Math.toRadians(90));
            else backDrop = new Pose2d(39, 54, Math.toRadians(90));
        } else if((indicator == LEFT) == (RobotConfig.teamColor == TeamColor.BLUE)){
            /*left for blue, right for red (furthest from the center)*/
            if(RobotConfig.teamColor == TeamColor.BLUE) backDrop = new Pose2d(-49, 52, Math.toRadians(90));
//            else backDrop = new Pose2d(46, 54, Math.toRadians(90));
            else backDrop = new Pose2d(44, 54, Math.toRadians(90));
        } else {
            //right for blue, left for red (closest to the center)
            if(RobotConfig.teamColor == TeamColor.BLUE) backDrop = new Pose2d(-28, 54, Math.toRadians(90));
//            else backDrop = new Pose2d(28, 54, Math.toRadians(90));
            else backDrop = new Pose2d(30, 54, Math.toRadians(90));
        }
    }

    void updateSideConfiguration(){
        if(gamepad1.b || gamepad2.b) RobotConfig.teamColor = TeamColor.RED;
        if(gamepad1.x || gamepad2.x) RobotConfig.teamColor = TeamColor.BLUE;
        if(gamepad1.dpad_right || gamepad2.dpad_right) RobotConfig.initialSide = InitialSide.RIGHT;
        if(gamepad1.dpad_left || gamepad2.dpad_left) RobotConfig.initialSide = InitialSide.LEFT;
        isRight = RobotConfig.initialSide == InitialSide.RIGHT;
        isRed = RobotConfig.teamColor == TeamColor.RED;
    }
}
