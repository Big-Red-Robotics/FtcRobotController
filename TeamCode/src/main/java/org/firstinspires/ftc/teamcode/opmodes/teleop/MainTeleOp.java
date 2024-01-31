package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.TouchSensor;
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
        arm.setLiftPosition(arm.ground);
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
                strafeSpeed = 1;
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

            //claw
//            if (gamepad2.left_trigger > 0) TODO: ENABLE AUTO GRAB
            if (gamepad2.right_trigger > 0) {
                arm.toggleClaw();
                sleep(300);
            }

            if(gamepad2.left_bumper || gamepad2.right_bumper){
                if (gamepad2.left_bumper) arm.toggleLeftClaw();
                if (gamepad2.right_bumper) arm.toggleRightClaw();

                sleep(200);
            }


            //lift
            if (gamepad2.a) {
                arm.setLiftPosition(arm.ground);
                arm.setRotatorLevel(0);
                arm.setClawFlip(false);
            }
            else if (gamepad2.x) {
                if(gamepad2.dpad_down || gamepad2.dpad_left){
                    if(gamepad2.dpad_down) arm.setLiftPosition(arm.low);
                    else arm.setLiftPosition(arm.middle);

                    arm.setClawFlip(true);
                    arm.setRotatorLevel(1);
                }
            } else if (gamepad2.y) {
                arm.setLiftPosition(arm.hang);
                arm.setClawFlip(false);
            }
            else if (gamepad2.b) {
                arm.setLiftPosition(arm.high);
                arm.setClawFlip(false);
            }

            //armEx
            if (!gamepad2.x && gamepad2.dpad_down) arm.setArmExtensionPosition(0);
            else if (!gamepad2.x && gamepad2.dpad_left) arm.setArmExtensionPosition(1000);
            else if (gamepad2.dpad_up) arm.setArmExtensionPosition(1600);
            //claw rotator
            else if (gamepad2.dpad_right) arm.toggleClawRotator();

            //manual lift
            if(gamepad2.left_stick_button) arm.resetLift();
            if(gamepad2.left_stick_y != 0.0) arm.setLiftPower(-0.5 * gamepad2.left_stick_y);
            else arm.update(true);

            //manual armEx
            if(gamepad2.right_stick_button) arm.resetArmExtension();
            if(gamepad2.right_stick_y != 0.0) arm.setArmExtensionPower(-0.5 * gamepad2.right_stick_y);

            //drone
            if (gamepad2.a || gamepad2.x) drone.home();
            else if (gamepad2.y) drone.launch();
            else if (gamepad2.b) drone.prepareLaunch();

            //telemetry
            telemetry.addData("arm position", arm.getLiftPosition());
            telemetry.addData("arm target", arm.getLiftTargetPosition());

            telemetry.addData("drone position", drone.positionDrone.getPosition());

            telemetry.addData("left claw position", arm.getLeftClawPosition());
            telemetry.addData("right claw position", arm.getRightClawPosition());

            telemetry.addData("pivot encoder", arm.getClawPivotPosition());
            telemetry.addData("rotator  encoder",arm.getRotatorPosition());


            telemetry.addData("ArmEx power", arm.getArmExPower());
            telemetry.addData("ArmEx position", arm.getArmExPosition());
            telemetry.addData("ArmEx target position", arm.getArmExTargetPosition());

            telemetry.update();

            if(isStopRequested()) drone.home();
        }
    }
}
