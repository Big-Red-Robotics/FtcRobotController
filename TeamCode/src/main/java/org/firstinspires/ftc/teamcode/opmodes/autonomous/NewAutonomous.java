package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.NewArm;
import org.firstinspires.ftc.teamcode.components.NewVision;
import org.firstinspires.ftc.teamcode.components.lib.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.teaminfo.InitialSide;
import org.firstinspires.ftc.teamcode.utility.teaminfo.TeamColor;

@Autonomous(name="2023-24 CenterStage")
public class NewAutonomous extends LinearOpMode {
    //indicator
    enum Indicator {RIGHT, MIDDLE, LEFT}
    Indicator indicator = Indicator.MIDDLE;

    //team info
    boolean isRight, isRed;

    //odometry spots
    Pose2d startPose, prePixel, dropPixel, park, backDrop;

    @Override
    public void runOpMode() {
        NewVision vision = new NewVision(hardwareMap);
        NewArm arm = new NewArm(hardwareMap);
        Chassis chassis = new Chassis(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            updateSideConfiguration();
            updateIndicator(vision);
            updateCoordinates();

            telemetry.addData("Team color", RobotConfig.teamColor);
            telemetry.addData("Initial side", RobotConfig.initialSide);
            telemetry.addData("Reading indicator", null);
            telemetry.addData("Detected indicator", indicator);
            telemetry.update();

            sleep(100);
        }

        //.waitSeconds
        //Trajectory Sequence
        chassis.setPoseEstimate(startPose);
        //adjust location to AprilTag
        //place pixels on backdrop
        TrajectorySequence traj1 = chassis.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(prePixel)
                .lineToLinearHeading(dropPixel)
                .build();

        Trajectory traj2 = chassis.trajectoryBuilder(dropPixel)
                .lineToLinearHeading(backDrop)
                .build();

        TrajectorySequence traj3 = chassis.trajectorySequenceBuilder(backDrop)
                .back(5)
                .lineToLinearHeading(park)
                .build();

        chassis.followTrajectorySequence(traj1);
        arm.openRightClaw();
        sleep(500);
        arm.setState(NewArm.ArmState.level1);
        arm.update();
        chassis.followTrajectory(traj2);
        arm.openLeftClaw();
        sleep(500);
        chassis.followTrajectorySequence(traj3);

    }

    void updateCoordinates(){
        boolean closeToBackdrop = isRight == isRed;
        int color = (isRed) ? 1 : -1;
        int side = (closeToBackdrop) ? 1 : -1;
        int initialHeading = (isRed) ? 180 : 0;
        startPose = new Pose2d(65 * color, 10 * side, Math.toRadians(initialHeading));
        prePixel = new Pose2d(36 * color, 24 * side, Math.toRadians((closeToBackdrop) ? -90 : initialHeading));
        //TODO: think about closeToBackdrop == false parking
        park = new Pose2d(62 * color, 45, Math.toRadians(-90));

        if(closeToBackdrop){
            switch(indicator){
                case LEFT:
                    dropPixel = new Pose2d(36 * color, 12, Math.toRadians(-90));
                    backDrop = new Pose2d(43 * color, 52, Math.toRadians(90));
                    break;
                case MIDDLE:
                    dropPixel = new Pose2d(24 * color, 24, Math.toRadians(-90));
                    backDrop = new Pose2d(35 * color, 52, Math.toRadians(90));
                    break;
                case RIGHT:
                    dropPixel = new Pose2d(36 * color, 36, Math.toRadians(-90));
                    backDrop = new Pose2d(27 * color, 52, Math.toRadians(90));
            }
        } else {
            //TODO
            if(indicator == Indicator.LEFT) dropPixel = new Pose2d(36 * color, 12, Math.toRadians(initialHeading));
            else if(indicator == Indicator.MIDDLE) dropPixel = new Pose2d(26 * color, 12, Math.toRadians(initialHeading));
            else if(indicator == Indicator.RIGHT) dropPixel = new Pose2d(36 * color, 12, Math.toRadians(initialHeading));
            backDrop = dropPixel;
        }
    }

    void updateIndicator(NewVision vision){
        int rawIndicatorValue = vision.getIndicator();
        switch (rawIndicatorValue){
            case 1:
                indicator = Indicator.LEFT;
                break;
            case 2:
                indicator = Indicator.MIDDLE;
                break;
            case 3:
                indicator = Indicator.RIGHT;
                break;
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
