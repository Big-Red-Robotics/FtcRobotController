package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.NewArm;
import org.firstinspires.ftc.teamcode.components.NewDrone;

@TeleOp(name="Dec 3 TeleOp")
public class FirstTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        //initialize components
        Chassis chassis = new Chassis(hardwareMap);
        double forwardSpeed, strafeSpeed, rotateSpeed;
        NewArm arm = new NewArm(hardwareMap);
        NewDrone drone = new NewDrone(hardwareMap);

        //log data
        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                forwardSpeed = 0.35;
                strafeSpeed = 0.25;
                rotateSpeed = 0.35;
            } else if (gamepad1.right_bumper) {
                forwardSpeed = 1;
                strafeSpeed = 0;
                rotateSpeed = 1;
            } else {
                forwardSpeed = 0.65;
                strafeSpeed = 0.5;
                rotateSpeed = 0.65;
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

            if (gamepad2.left_bumper) arm.openClaw();
            if (gamepad2.dpad_left) arm.openLeftClaw();
            if (gamepad2.dpad_right) arm.openRightClaw();
            if (gamepad2.right_bumper) arm.closeClaw();

            /*arm.leftClaw.setPosition(0.5*gamepad2.left_stick_x+0.5);
            arm.rightClaw.setPosition(0.5*gamepad2.right_stick_x+0.5);*/

            if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) arm.setLiftPower(gamepad2.right_trigger*.75 - gamepad2.left_trigger*.5, telemetry);
            else arm.update();

            if (gamepad2.a) {arm.setState(NewArm.ArmState.intake); drone.home();}
            if (gamepad2.x) arm.setState(NewArm.ArmState.outtake);
            if (gamepad2.y) {arm.setState(NewArm.ArmState.hang); drone.prepareLaunch();}
            if (gamepad2.b && arm.hang) drone.launch();

            if (gamepad2.dpad_up) arm.flipWrist();
            if (gamepad2.dpad_down) arm.closeWrist();

            telemetry.addData("arm position", arm.getLiftPosition());
            telemetry.addData("current mode", arm.currentState);
            telemetry.addData("drone position", drone.positionDrone.getPosition());
            telemetry.addData("arm target", arm.getTargetPosition());
            telemetry.addData("left claw position", arm.leftClaw.getPosition());
            telemetry.addData("right claw position", arm.rightClaw.getPosition());

            telemetry.addData("wrist encoder",arm.clawRotator.getPosition());

            telemetry.update();
        }
    }
}
