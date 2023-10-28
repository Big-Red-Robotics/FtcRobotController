package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.old.Arm;
import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.old.Vision;
import org.firstinspires.ftc.teamcode.components.lib.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.teaminfo.InitialSide;
import org.firstinspires.ftc.teamcode.utility.teaminfo.TeamColor;

@Disabled
@Autonomous(name="2022-23 Powerplay")
public class TestAutonomous extends LinearOpMode {
    enum State {
        START,
        TRAJ_1,
        LIFT_ARM,
        GO_TO_POLE,
        SCORE_CONE,
        PARK,
        WAIT
    }
    State currentState = State.WAIT;

    Pose2d startPose = new Pose2d(36, -60, Math.toRadians(90));

    Vector2d nearPole = new Vector2d(36, -15);
    Vector2d parkRight = new Vector2d(60, -12);
    Vector2d parkMiddle = new Vector2d(36, -12);
    Vector2d parkLeft = new Vector2d(12, -12);
    Vector2d park = parkMiddle;
    boolean parked = false;

    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        arm.armTarget = 0;
        Vision vision = new Vision(hardwareMap);

        TrajectorySequence traj1 = chassis.trajectorySequenceBuilder(startPose)
                .forward(50)
                .back(5)
                .turn(Math.toRadians(40))
                .build();

        TrajectorySequence traj2 = chassis.trajectorySequenceBuilder(chassis.getPoseEstimate())
                .forward(7)
                .build();

        TrajectorySequence traj3 = chassis.trajectorySequenceBuilder(traj2.end())
                .back(5)
                .turn(Math.toRadians(-40))
                .strafeTo(nearPole)
                .strafeTo(park)
                .build();

        while (!isStarted() && !isStopRequested()) {
            vision.searchTags();

            if(gamepad1.b || gamepad2.b) RobotConfig.teamColor = TeamColor.RED;
            if(gamepad1.x || gamepad2.x) RobotConfig.teamColor = TeamColor.BLUE;
            if(gamepad1.dpad_right || gamepad2.dpad_right) RobotConfig.initialSide = InitialSide.RIGHT;
            if(gamepad1.dpad_left || gamepad2.dpad_left) RobotConfig.initialSide = InitialSide.LEFT;

            if(vision.tagId() > 0){
                telemetry.addData("Detected tag ID:", vision.tagId());
                switch(vision.tagId()){
                    case 1:
                        park = parkLeft;
                        break;
                    case 2:
                        park = parkMiddle;
                        break;
                    case 3:
                        park = parkRight;
                        break;
                    default:
                        break;
                }
            }
            else telemetry.addLine("Don't see tag of interest :(");

            telemetry.addData("parking location", park);
            telemetry.addData("Team color", RobotConfig.teamColor);
            telemetry.addData("Initial side", RobotConfig.initialSide);
            telemetry.addData("distance", vision.distance());
            telemetry.update();

            sleep(20);
        }

        chassis.setPoseEstimate(startPose);
        currentState = State.START;
        //TODO: arm.closeGripper();

        while(opModeIsActive() && !parked) {
            switch (currentState){
                case START:
                    if (!chassis.isBusy()) {
                        currentState = State.TRAJ_1;
                        chassis.followTrajectorySequenceAsync(traj1);
                    }
                    break;
                case TRAJ_1:
                    if (!chassis.isBusy()) {
                        currentState = State.LIFT_ARM;
                        //TODO: arm.runToPosition(arm.highJunction);
                    }
                    break;
                case LIFT_ARM:
                    if (!chassis.isBusy()){
                        currentState = State.GO_TO_POLE;
                        chassis.followTrajectorySequenceAsync(traj2);
                    }
                    break;
                case GO_TO_POLE:
                    if (!chassis.isBusy()){
                        currentState = State.SCORE_CONE;
                        //TODO: arm.fall();
                        //      arm.openGripper();
                    }
                case SCORE_CONE:
                    if (!chassis.isBusy()){
                        currentState = State.PARK;
                        chassis.followTrajectorySequenceAsync(traj3);
                    }
                    break;
                case PARK:
                    if (!chassis.isBusy()){
                        currentState = State.WAIT;
                        chassis.followTrajectorySequenceAsync(traj3);
                    }
                    break;
                case WAIT:
                    break;
            }
        }
    }

    void adjust(Chassis chassis, Vision vision, int mode){
        final int rotate = 0;
        final int strafe = 1;
        while(Math.abs(vision.getAutonPipeline().differenceX()) > 20) {
            double direction = Math.abs(vision.getAutonPipeline().differenceX())/vision.getAutonPipeline().differenceX();
            double power = (Math.abs(vision.getAutonPipeline().differenceX()) > 50) ? 0.3 * direction : 0.1 * direction;
            if(mode == rotate) chassis.rotate(power);
            if(mode == strafe) chassis.strafe(power);
            telemetry.addData("difference", vision.getAutonPipeline().differenceX());
            telemetry.update();
        }
    }
}
