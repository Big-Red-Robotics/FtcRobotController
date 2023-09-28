package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.NewVision;
import org.firstinspires.ftc.teamcode.components.Vision;
import org.firstinspires.ftc.teamcode.components.lib.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.teaminfo.InitialSide;
import org.firstinspires.ftc.teamcode.utility.teaminfo.TeamColor;

@Autonomous(name="2023-24 CenterStage")
public class NewAutonomous extends LinearOpMode {
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

    //configuration for Blue Alliance Far Side (closer to the drone landing zone)
    Pose2d startPose = new Pose2d(-60, -36, Math.toRadians(0));
    Pose2d backDrop = new Pose2d(-36, 50, Math.toRadians(90));
    Pose2d stack1 = new Pose2d(-36, -50, Math.toRadians(-90));

    boolean done = false;

    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis(hardwareMap);
        NewVision vision = new NewVision(hardwareMap);

        TrajectorySequence startToBackDrop = chassis.trajectorySequenceBuilder(startPose)
                .strafeLeft(48)
                .lineToLinearHeading(backDrop)
                .build();

        TrajectorySequence backDropToStack = chassis.trajectorySequenceBuilder(backDrop)
                .lineToLinearHeading(stack1)
                .build();

        while (!isStarted() && !isStopRequested()) {
            if(gamepad1.b || gamepad2.b) RobotConfig.teamColor = TeamColor.RED;
            if(gamepad1.x || gamepad2.x) RobotConfig.teamColor = TeamColor.BLUE;
            //TODO: Adjust the name of initial side
            if(gamepad1.dpad_right || gamepad2.dpad_right) RobotConfig.initialSide = InitialSide.RIGHT;
            if(gamepad1.dpad_left || gamepad2.dpad_left) RobotConfig.initialSide = InitialSide.LEFT;

            //TODO: Scan if the middle line has an indicator

            telemetry.addData("Team color", RobotConfig.teamColor);
            telemetry.addData("Initial side", RobotConfig.initialSide);
            telemetry.update();

            sleep(20);
        }

        chassis.setPoseEstimate(startPose);
        currentState = State.START;
        //TODO: arm.closeGripper();

        while(opModeIsActive() && !done) {
            switch (currentState){
                case START:
                    if (!chassis.isBusy()) {
                        currentState = State.TRAJ_1;
//                        chassis.followTrajectorySequenceAsync(traj1);
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
//                        chassis.followTrajectorySequenceAsync(traj2);
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
//                        chassis.followTrajectorySequenceAsync(traj3);
                    }
                    break;
                case PARK:
                    if (!chassis.isBusy()){
                        currentState = State.WAIT;
//                        chassis.followTrajectorySequenceAsync(traj3);
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
