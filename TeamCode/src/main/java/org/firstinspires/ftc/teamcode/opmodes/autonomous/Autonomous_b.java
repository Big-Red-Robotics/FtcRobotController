package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.Vision;
import org.firstinspires.ftc.teamcode.components.lib.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.components.lib.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.teaminfo.InitialSide;
import org.firstinspires.ftc.teamcode.utility.teaminfo.TeamColor;

@Autonomous(name="1+0 FARSIDE", preselectTeleOp = "2023-2024 CENTERSTAGE")
public class Autonomous_b extends LinearOpMode {
    //indicator
    int indicator;
    int LEFT = 1, MIDDLE = 2, RIGHT = 3;

    //team info
    boolean isRight, isRed;
    boolean isCloseToBackdrop;

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

            arm.setArmExtensionPosition(0);

            if (gamepad1.right_bumper) arm.resetLift();

            telemetry.addData("Team color", RobotConfig.teamColor);
            telemetry.addData("Initial side", RobotConfig.initialSide);
            telemetry.addData("Detected indicator", indicator);
            telemetry.update();

            sleep(100);
        }

        //put claw down (claw flipped up for initialization due to 18-inch restriction)
        arm.setClawRotatorPosition(0.40);
        arm.setArmExtensionPosition(15);
        if (isRight == isRed) waitSeconds(1.0);
//        else waitSeconds(8.5);

        //update indicator information
        indicator = vision.getIndicator();
        updateCoordinates();
        telemetry.addData("Detected indicator", indicator);
        telemetry.update();

        chassis.setPoseEstimate(startPose);

        //ALL, startPosition ~ pixel location
        TrajectorySequence startToPixel = chassis.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(prePixel)
                .lineToLinearHeading(dropPixel)
                .build();

        //CLOSE TO BACKDROP, dropped pixel ~ backdrop
        TrajectorySequence closesideToPreBackdrop = chassis.trajectorySequenceBuilder(dropPixel)
                .back(5)
                .lineToSplineHeading(intermediate)
                .build();

        TrajectorySequence closesideToBackdrop = chassis.trajectorySequenceBuilder(intermediate)
                .lineToSplineHeading(backDrop, Chassis.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Chassis.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence farsideEnd = chassis.trajectorySequenceBuilder(dropPixel)
                .back(5)
                .build();

        //UNIVERSAL backdrop ~ park
        TrajectorySequence backdropToPark = chassis.trajectorySequenceBuilder(backDrop)
                .back(10,
                        Chassis.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Chassis.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(park)
                .build();

        //actual autonomous sequence
        chassis.followTrajectorySequence(startToPixel);
        if (RobotConfig.teamColor == TeamColor.RED) arm.openRightClaw();
        else arm.openLeftClaw();
        waitSeconds(0.6);
        arm.setClawRotatorPosition(2);
        waitSeconds(0.5);

        if (isRight == isRed) {
            //go to backdrop
            chassis.followTrajectorySequence(closesideToPreBackdrop);
            arm.toPosition(Arm.AUTON, 1, false, telemetry);
            arm.toPosition(Arm.AUTON, 1, true, telemetry);
            chassis.followTrajectorySequence(closesideToBackdrop);
            //at backdrop
            if (RobotConfig.teamColor == TeamColor.RED) arm.openLeftClaw();
            else arm.openRightClaw();
            waitSeconds(0.6);
            chassis.followTrajectorySequence(backdropToPark);
            arm.closeClaw();
            arm.toPosition(Arm.AUTON, 2, false, telemetry);
            waitSeconds(0.5);
            arm.toPosition(Arm.GROUND, 2, false, telemetry);
        } else {
            chassis.followTrajectorySequence(farsideEnd);
            arm.toPosition(Arm.GROUND, 2, false, telemetry);
        }
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
        //final position
        if(isRed) park = new Pose2d(10, 50, Math.toRadians(90));
        else park = new Pose2d(-15, 50, Math.toRadians(90));

        if(closeToBackdrop){
            //CLOSE TO BACKDROP
//            park = new Pose2d(62 * color, 47, Math.toRadians(90)); //final position, corner
            prePixel = new Pose2d(36 * color, 24, Math.toRadians(-90)); //middle position before first pixel
            intermediate = new Pose2d(30 * color, 40, Math.toRadians(90));
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
            //WING SIDE
//            park = new Pose2d(10 * color, 47, Math.toRadians(90)); //final position
            prePixel = new Pose2d(43 * color, -34, Math.toRadians(initialHeading)); //middle position before first pixel
            if(indicator == MIDDLE) {
                dropPixel = new Pose2d(38 * color, -33, Math.toRadians(initialHeading));
                intermediate = new Pose2d((isRed) ? 48 : -48, -50, Math.toRadians((isRed) ? 180 : 0));
            } else if((indicator == LEFT) == (RobotConfig.teamColor == TeamColor.BLUE)) {
                /*left for blue, right for red (closest to the backdrop)*/
                dropPixel = new Pose2d(38 * color, -35, Math.toRadians(initialHeading - (50*color)));
                intermediate = new Pose2d((isRed) ? 48 : -48, -46, Math.toRadians((isRed) ? 180 : 0));
            } else {
                //right for blue, left for red (furthest from the backdrop)
                dropPixel = new Pose2d(44 * color, -51, Math.toRadians(initialHeading));
                if(isRed) intermediate = new Pose2d(48, -33, Math.toRadians(180));
                else intermediate = new Pose2d(-48, -32, Math.toRadians(0));
            }
        }

        if(indicator == MIDDLE){
            /*middle for both colors*/
            if(RobotConfig.teamColor == TeamColor.BLUE) backDrop = new Pose2d(-41, 54, Math.toRadians(90));
            else backDrop = new Pose2d(41, 54, Math.toRadians(90));
        } else if((indicator == LEFT) == (RobotConfig.teamColor == TeamColor.BLUE)){
            /*left for blue, right for red (closest to the backdrop)*/
            if(RobotConfig.teamColor == TeamColor.BLUE) backDrop = new Pose2d(-49, 54, Math.toRadians(90));
            else backDrop = new Pose2d(44, 54, Math.toRadians(90));
        } else {
            //right for blue, left for red (furthest from the backdrop)
            if(RobotConfig.teamColor == TeamColor.BLUE) backDrop = new Pose2d(-30, 54, Math.toRadians(90));
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
