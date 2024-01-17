package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Drone;

@TeleOp(name="Oct 29 TeleOp")
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
                            gamepad1.left_stick_y * forwardSpeed,
                            -gamepad1.left_stick_x * strafeSpeed,
                            -gamepad1.right_stick_x * rotateSpeed
                    )
            );
            chassis.update();

            if (gamepad2.left_bumper) arm.openClaw();
            if (gamepad2.right_bumper) arm.closeClaw();

            if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) arm.setLiftPower(gamepad2.right_trigger*.75 - gamepad2.left_trigger*.5);

            if (gamepad2.a) arm.setState(Arm.ArmState.intake);
            if (gamepad2.x) arm.setState(Arm.ArmState.outtake);
            if (gamepad2.y) arm.setState(Arm.ArmState.hang);

            if (gamepad2.dpad_up) {arm.setState(Arm.ArmState.hang); drone.prepareLaunch();}
            if (gamepad2.dpad_left || gamepad2.dpad_right) drone.launch();
            arm.update();

            telemetry.addData("arm position", arm.getLiftPosition());
            telemetry.addData("current mode", arm.currentState);
            telemetry.addData("servo position", arm.claw.getPosition());
            telemetry.addData("drone position", drone.positionDrone.getPosition());

            telemetry.addData("wrist encoder",arm.clawRotator.getPosition());

            telemetry.update();
        }
    }
}
