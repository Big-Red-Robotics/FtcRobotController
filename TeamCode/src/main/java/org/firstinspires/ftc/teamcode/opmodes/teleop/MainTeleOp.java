package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Drone;

@TeleOp(name="Dec 3 TeleOp")
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
            arm.clawPivot.setPosition(gamepad2.right_stick_y * 0.5 + 0.5);

            if (gamepad2.left_trigger > 0) arm.closeRightClaw();
            if (gamepad2.right_trigger > 0) arm.closeLeftClaw();
            if (gamepad2.right_bumper) arm.openLeftClaw();
            if (gamepad2.left_bumper) arm.openRightClaw();

            if (gamepad2.a) {arm.setState(Arm.ArmState.intake); arm.intake = 0; drone.home(); arm.pivot = false;}
            else if (gamepad2.x) {arm.setState(Arm.ArmState.outtake); arm.intake = 2; arm.pivot = false;}
            else if (gamepad2.y) {arm.setState(Arm.ArmState.hang); drone.prepareLaunch(); arm.intake = 2; arm.pivot = false;}
            else if (gamepad2.b && arm.hang) drone.launch();

            if (gamepad2.dpad_up) {arm.closeClaw(); sleep(200); arm.intake = 1;}
            else if (gamepad2.dpad_down) arm.openClaw();
            if (gamepad2.dpad_left) {arm.setState(Arm.ArmState.level1); arm.intake = 1; arm.pivot = true;}

            if (gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0) arm.setLiftPower(gamepad2.right_trigger * 0.75 - gamepad2.left_trigger * 0.5);
            else arm.update();

            telemetry.addData("arm position", arm.getLiftPosition());
            telemetry.addData("current mode", arm.getCurrentState());
            telemetry.addData("drone position", drone.positionDrone.getPosition());
            telemetry.addData("arm target", arm.getTargetPosition());
            telemetry.addData("left claw position", arm.leftClaw.getPosition());
            telemetry.addData("right claw position", arm.rightClaw.getPosition());
            telemetry.addData("rotator encoder", arm.clawPivot.getPosition());
            telemetry.addData("wrist encoder",arm.clawRotator.getPosition());

            telemetry.update();
        }
    }
}
