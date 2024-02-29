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

@Autonomous(name="2+1 FARSIDE", preselectTeleOp = "2023-2024 CENTERSTAGE")
public class AutonomousNew extends LinearOpMode {
    //indicator
    int indicator;
    int LEFT = 1, MIDDLE = 2, RIGHT = 3;

    //team info
    boolean isRight, isRed;
    boolean isCloseToBackdrop;

    //odometry spots
    Pose2d startPose, prePixel, dropPixel, park, backDrop, intermediate, grabStack;

    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Chassis chassis = new Chassis(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            indicator = vision.getIndicator();
            updateSideConfiguration();

            arm.setArmExtensionPosition(0);

            if(gamepad1.right_bumper) arm.resetLift();

            telemetry.addData("Team color", RobotConfig.teamColor);
            telemetry.addData("Initial side", RobotConfig.initialSide);
            telemetry.addData("Detected indicator", indicator);
            telemetry.update();

            sleep(100);
        }

        //put claw down (claw flipped up for initialization due to 18-inch restriction)
        arm.moveClawRotator(0);
        arm.setArmExtensionPosition(15);
        waitSeconds(1.0);

        //update indicator information
        indicator = vision.getIndicator();
        updateCoordinates();
        telemetry.addData("Detected indicator", indicator);
        telemetry.update();

        chassis.setPoseEstimate(startPose);

        //ALL, startPosition ~ pixel location
        //CAUTION: DOESN'T WORK THE BEST FOR CLOSE SIDE
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

        //FAR FROM BACKDROP, spike mark ~ pixel
        TrajectorySequence pixelToStackA = chassis.trajectorySequenceBuilder(dropPixel)
                .back(7)
                .lineToLinearHeading(intermediate)
                .build();

        TrajectorySequence pixelToStackB = chassis.trajectorySequenceBuilder(intermediate)
                .lineToLinearHeading(grabStack)
                .build();

        //FAR FROM BACKDROP, dropped pixel ~ backdrop
        TrajectorySequence farsideToPreBackdropA = chassis.trajectorySequenceBuilder(grabStack)
                .lineToLinearHeading(startPose)
                .build();

        int forward = (isRed) ? 5 : 3;
        TrajectorySequence farsideToPreBackdropB = chassis.trajectorySequenceBuilder(startPose)
                .forward(forward)
                .turn(Math.toRadians((isRed) ? -90 : 90))
                .forward(50)
                .splineTo(new Vector2d((isRed) ? 35 : -35, 45), Math.toRadians(90))
                .build();

        TrajectorySequence farsideToBackdrop = chassis.trajectorySequenceBuilder(new Pose2d((isRed) ? 35 : -35, 45, Math.toRadians(90)))
                .lineToSplineHeading(backDrop)
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
        if(RobotConfig.teamColor == TeamColor.RED) arm.openRightClaw();
        else arm.openLeftClaw();
        waitSeconds(0.5);
        arm.moveClawRotator(2);

        //go to backdrop
        if(isCloseToBackdrop)chassis.followTrajectorySequence(closesideToPreBackdrop);
        else {
            //arm lift to an appropriate position and claw opened so that it can actually grab.
            chassis.followTrajectorySequence(pixelToStackA);

            arm.moveClawRotator(4);
            arm.toPosition(135, 4, false, telemetry);
            waitSeconds(0.5);

            chassis.followTrajectorySequence(pixelToStackB);

            //grab pixel into the appropriate claw
            if(RobotConfig.teamColor == TeamColor.RED) arm.closeRightClaw();
            else arm.closeLeftClaw();
            waitSeconds(0.5);

            chassis.followTrajectorySequence(farsideToPreBackdropA);
            arm.moveClawRotator(2);
            arm.toPosition(Arm.GROUND, 2, false, telemetry);
            chassis.followTrajectorySequence(farsideToPreBackdropB);
        }

        arm.moveClawRotator(1);
        arm.toPosition(Arm.AUTON, 1,false, telemetry);
        arm.toPosition(Arm.AUTON, 1,true, telemetry);

        if(isCloseToBackdrop) chassis.followTrajectorySequence(closesideToBackdrop);
        else chassis.followTrajectorySequence(farsideToBackdrop);

        //at backdrop
//        if(RobotConfig.teamColor == TeamColor.RED) arm.openLeftClaw();
//        else arm.openRightClaw();
        arm.openClaw(); //TODO: actually open claw one by one so that both scores appropriately
        waitSeconds(0.6);
        chassis.followTrajectorySequence(backdropToPark);
        arm.closeClaw();
        arm.toPosition(Arm.AUTON, 2,false, telemetry);
        waitSeconds(0.5);
        arm.toPosition(Arm.GROUND, 2, false, telemetry);
    }

    void waitSeconds(double seconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds() < seconds);
    }

    void updateCoordinates(){
        //TODO: run trials on blue side & adjust all location
        isCloseToBackdrop = isRight == isRed;
        int color = (isRed) ? 1 : -1;
        int initialHeading = (isRed) ? 180 : 0;
        startPose = new Pose2d(65 * color, (isCloseToBackdrop) ? 10 : -34, Math.toRadians(initialHeading));
        //final position
//        if(isRed) park = new Pose2d(20, 50, Math.toRadians(90));
//        else park = new Pose2d(-20, 50, Math.toRadians(90));
        //stack position
        if(isRed) grabStack = new Pose2d(38, -53, Math.toRadians(-90));
        else grabStack = new Pose2d(-41, -54.5, Math.toRadians(-90));

        if(isCloseToBackdrop){
            //CLOSE TO BACKDROP
            park = new Pose2d(10 * color, 47, Math.toRadians(90)); //final position
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
            //TODO, farside
            park = new Pose2d(60 * color, 47, Math.toRadians(90)); //final position, corner
            prePixel = new Pose2d(43 * color, -34, Math.toRadians(initialHeading)); //middle position before first pixel
            intermediate = new Pose2d((isRed) ? 40 : -40, -35, Math.toRadians(-90));
            if(indicator == MIDDLE) {
                dropPixel = new Pose2d(38 * color, -33, Math.toRadians(initialHeading));
            } else if((indicator == LEFT) == (RobotConfig.teamColor == TeamColor.BLUE)) {
                /*left for blue, right for red (closest to the backdrop)*/
                dropPixel = new Pose2d(38 * color, -35, Math.toRadians(initialHeading - (50*color)));
            } else {
                //right for blue, left for red (furthest from the backdrop)
                dropPixel = new Pose2d(38 * color, -37.5, Math.toRadians(initialHeading + (50*color)));
            }
        }

        //backdrop
        //TODO: adjust
        if(indicator == MIDDLE){
            /*middle for both colors*/
            if(RobotConfig.teamColor == TeamColor.BLUE) backDrop = new Pose2d(-38, 56, Math.toRadians(90));
            else backDrop = new Pose2d(28, 55, Math.toRadians(90));
        } else if((indicator == LEFT) == (RobotConfig.teamColor == TeamColor.BLUE)){
            /*left for blue, right for red (closest to the backdrop)*/
            if(RobotConfig.teamColor == TeamColor.BLUE) backDrop = new Pose2d(-48, 56, Math.toRadians(90));
            else backDrop = new Pose2d(38, 55, Math.toRadians(90));
        } else {
            //right for blue, left for red (furthest from the backdrop)
            if(RobotConfig.teamColor == TeamColor.BLUE) backDrop = new Pose2d(-26, 56, Math.toRadians(90));
            else backDrop = new Pose2d(18, 55, Math.toRadians(90));
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