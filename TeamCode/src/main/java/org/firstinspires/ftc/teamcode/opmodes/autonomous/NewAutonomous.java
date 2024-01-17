package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.Vision;
import org.firstinspires.ftc.teamcode.components.lib.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.teaminfo.InitialSide;
import org.firstinspires.ftc.teamcode.utility.teaminfo.TeamColor;

@Disabled
@Autonomous(name="2023-24 CenterStage")
public class NewAutonomous extends LinearOpMode {
    //configuration for Blue Alliance Far Side (closer to the drone landing zone)
    Pose2d startPose = new Pose2d(-60, -36, Math.toRadians(0));
    Pose2d backDrop = new Pose2d(-36, 50, Math.toRadians(90));
    Pose2d stack1 = new Pose2d(-36, -50, Math.toRadians(-90));
    Pose2d park = new Pose2d(-36, -10, Math.toRadians(-90));

    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis(hardwareMap);
        Vision vision = new Vision(hardwareMap);

        //.waitSeconds
        TrajectorySequence traj = chassis.trajectorySequenceBuilder(startPose)
                .strafeLeft(48)
                .lineToLinearHeading(backDrop)
                .addDisplacementMarker(() -> {
                    //adjust location to AprilTag
                    //place pixels on backdrop
                })
                .lineToLinearHeading(stack1)
                .addDisplacementMarker(() -> {
                    //pickup pixels from stack
                })
                .lineToLinearHeading(backDrop)
                .addDisplacementMarker(() -> {
                    //place pixels on backdrop
                })
                .lineToLinearHeading(park)
                .addDisplacementMarker(() -> {
                    //park
                })
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
            telemetry.addData("Reading indicator", null);
            telemetry.update();

            sleep(20);
        }

        while(opModeIsActive()) {
            chassis.followTrajectorySequenceAsync(traj);
        }
    }

    void adjust(Chassis chassis, org.firstinspires.ftc.teamcode.components.old.Vision vision, int mode){
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
