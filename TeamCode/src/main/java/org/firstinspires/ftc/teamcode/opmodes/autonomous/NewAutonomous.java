package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.NewArm;
import org.firstinspires.ftc.teamcode.components.NewVision;
import org.firstinspires.ftc.teamcode.components.lib.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.components.lib.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.teaminfo.InitialSide;
import org.firstinspires.ftc.teamcode.utility.teaminfo.TeamColor;

@Autonomous(name="2023-24 CenterStage")
public class NewAutonomous extends LinearOpMode {
    //indicator
    enum Indicator {RIGHT, MIDDLE, LEFT}
    Indicator indicator;

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

            telemetry.addData("Team color", RobotConfig.teamColor);
            telemetry.addData("Initial side", RobotConfig.initialSide);
            telemetry.addData("Detected indicator", indicator);
            telemetry.update();

            sleep(100);
        }

        //put claw down (claw flipped up for initialization due to 18-inch restriction)
        arm.setClawRotatorPosition(0.43);
        waitSeconds(1.5);

        //update indicator information
        updateIndicator(vision);
        updateCoordinates();
        telemetry.addData("Detected indicator", indicator);
        telemetry.update();

        //path building
        chassis.setPoseEstimate(startPose);

        TrajectorySequence traj1 = chassis.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(prePixel)
                .lineToLinearHeading(dropPixel)
                .build();

        TrajectorySequence traj2 = chassis.trajectorySequenceBuilder(dropPixel)
                .back(5)
                .addDisplacementMarker(() -> {
                    arm.toPosition(NewArm.ArmState.level1);
                })
                .lineToLinearHeading(backDrop)
                .build();

        TrajectorySequence traj3 = chassis.trajectorySequenceBuilder(backDrop)
                .back(7,
                        Chassis.getVelocityConstraint(3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Chassis.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(park)
                .build();

        //actual autonomous sequence
        chassis.followTrajectorySequence(traj1);
        if(RobotConfig.teamColor == TeamColor.BLUE) arm.openRightClaw();
        else arm.openLeftClaw();
        waitSeconds(1.5);
        chassis.followTrajectorySequence(traj2);
        arm.openLeftClaw();
        if(RobotConfig.teamColor == TeamColor.BLUE) arm.openLeftClaw();
        else arm.openRightClaw();
        waitSeconds(1.5);
        chassis.followTrajectorySequence(traj3);

    }

    void waitSeconds(double seconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds() < seconds) ;
    }

    void updateCoordinates(){
        boolean closeToBackdrop = isRight == isRed;
        int color = (isRed) ? 1 : -1;
        int side = (closeToBackdrop) ? 1 : -1;
        int initialHeading = (isRed) ? 180 : 0;
        startPose = new Pose2d(65 * color, 10 * side,
                Math.toRadians(initialHeading));
        prePixel = new Pose2d(36 * color, 24 * side, Math.toRadians((closeToBackdrop) ? -90 : initialHeading));
        //TODO: think about closeToBackdrop == false parking
        park = new Pose2d(62 * color, 45, Math.toRadians(-90));

        if(closeToBackdrop){
            if(indicator == Indicator.MIDDLE){
                dropPixel = new Pose2d(25 * color, 24, Math.toRadians(-90));
                if(RobotConfig.teamColor == TeamColor.BLUE) backDrop = new Pose2d(-44, 54, Math.toRadians(90));
                else backDrop = new Pose2d(33, 54, Math.toRadians(90));
            } else if((indicator == Indicator.LEFT) == (RobotConfig.teamColor == TeamColor.BLUE)){
                //left for blue, right for red (furthest from the center)
                dropPixel = new Pose2d(32 * color, 35, Math.toRadians(-90));
                if(RobotConfig.teamColor == TeamColor.BLUE) backDrop = new Pose2d(-53, 54, Math.toRadians(90));
                else backDrop = new Pose2d(43, 54, Math.toRadians(90));
            } else {
                //right for blue, left for red (closes to the center)
                dropPixel = new Pose2d(32 * color, 12, Math.toRadians(-90));
                if(RobotConfig.teamColor == TeamColor.BLUE) backDrop = new Pose2d(-37, 54, Math.toRadians(90));
                else backDrop = new Pose2d(29, 54, Math.toRadians(90));
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
