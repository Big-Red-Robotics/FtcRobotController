package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.analysis.function.Abs;
import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Drone;

@TeleOp(name="Jan 14 TeleOp")
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        //initialize components
        Chassis chassis = new Chassis(hardwareMap);
        double forwardSpeed, strafeSpeed, rotateSpeed;
        Arm arm = new Arm(hardwareMap);
        Drone drone = new Drone(hardwareMap);

        //log data
        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            //chassis
            if (gamepad1.left_bumper) {
                forwardSpeed = 0.4;
                strafeSpeed = 0.4;
                rotateSpeed = 0.4;
            } else {
                forwardSpeed = 1;
                strafeSpeed = 0.8;
                rotateSpeed = 0.9;
            }

            double left_y = gamepad1.left_stick_y;
            double left_x = gamepad1.left_stick_x;
            if(Math.abs(Math.atan2(Math.abs(left_x), Math.abs(left_y))) < Math.PI/6.0 && left_y != 0){
                strafeSpeed = 0;
            } else if(Math.abs(Math.atan2(Math.abs(left_y), Math.abs(left_x))) < Math.PI/6.0 && left_x != 0){
                forwardSpeed = 0;
            }
            chassis.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * forwardSpeed,
                            -gamepad1.left_stick_x * strafeSpeed,
                            -gamepad1.right_stick_x * rotateSpeed
                    )
            );
            chassis.update();

            //arm
            if (gamepad2.left_trigger > 0) arm.closeRightClaw();
            if (gamepad2.right_trigger > 0) arm.closeLeftClaw();

            if (gamepad2.left_bumper) arm.openLeftClaw();
            if (gamepad2.right_bumper) arm.openRightClaw();

            if (gamepad2.a) {
                arm.closeClaw();
                arm.setState(Arm.ArmState.ground, 2, false);
            } else if (gamepad2.y) arm.setState(Arm.ArmState.hang, 2, false);
            else if (gamepad2.x) arm.setState(Arm.ArmState.high, 2, false);
            else if (gamepad2.b && arm.hang) drone.launch();

            if (gamepad2.dpad_left) arm.setState(Arm.ArmState.low, 1, true);
            else if (gamepad2.dpad_down) arm.openClaw();
            else if (gamepad2.dpad_right) {arm.setState(Arm.ArmState.ground, 0, false); arm.openClaw();}
            else if (gamepad2.dpad_up) {
                arm.closeClaw();
                sleep(200);
                arm.setState(arm.getCurrentState(), 2, arm.getClawFlip());
            }

            if(gamepad2.left_stick_button) arm.reset();

            if(gamepad2.left_stick_y != 0.0) arm.setLiftPower(-0.5 * gamepad2.left_stick_y);
            else arm.update(true);

            //drone
            if (gamepad2.a) drone.home();
            else if (gamepad2.y) drone.prepareLaunch();
            else if (gamepad2.b && arm.hang) drone.launch();

            //telemetry
            telemetry.addData("arm position", arm.getLiftPosition());
            telemetry.addData("current mode", arm.getCurrentState());
            telemetry.addData("drone position", drone.positionDrone.getPosition());
            telemetry.addData("arm target", arm.getTargetPosition());
            telemetry.addData("left claw position", arm.leftClaw.getPosition());
            telemetry.addData("right claw position", arm.rightClaw.getPosition());
            telemetry.addData("rotator encoder", arm.clawPivot.getPosition());
            telemetry.addData("wrist encoder",arm.clawRotator.getPosition());

            telemetry.update();

            if(isStopRequested()) drone.home();
            Hitec
        }
    }
}
