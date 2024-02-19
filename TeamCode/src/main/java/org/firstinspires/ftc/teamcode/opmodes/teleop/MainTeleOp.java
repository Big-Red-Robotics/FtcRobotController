package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Drone;
import org.firstinspires.ftc.teamcode.components.PixelIndicator;

@TeleOp(name="2023-2024 CENTERSTAGE")
public class MainTeleOp extends LinearOpMode {
    RevBlinkinLedDriver.BlinkinPattern pattern;

    @Override
    public void runOpMode() {
        //initialize components
        Chassis chassis = new Chassis(hardwareMap);
        double forwardSpeed, strafeSpeed, rotateSpeed;
        Arm arm = new Arm(hardwareMap);
        arm.setLiftPosition(Arm.GROUND);
        Drone drone = new Drone(hardwareMap);
        PixelIndicator pixelIndicator = new PixelIndicator(hardwareMap);

        //states
        int droneState = 0;
        int continuousArmControlState = 0;

        //log data
        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        arm.setArmExtensionPosition(0);

        while (opModeIsActive()) {
            boolean delay = false;
            droneState = droneState % 3;

            //chassis
            if (gamepad1.left_bumper) {
                forwardSpeed = 0.4;
                strafeSpeed = 0.7;
                rotateSpeed = 0.4;
            } else {
                forwardSpeed = 1;
                strafeSpeed = 1.5;
                rotateSpeed = 0.7;
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

            //auto grab
            if (gamepad1.right_bumper){
                arm.setRotatorLevel(0);
                arm.openClaw();
                if(pixelIndicator.isTherePixelL()) arm.closeLeftClaw();
                if(pixelIndicator.isTherePixelR()) arm.closeRightClaw();
            }

            //auto re-grab
            if(gamepad1.right_trigger > 0){
                ElapsedTime timer = new ElapsedTime();
                timer.reset();
                if(!pixelIndicator.isTherePixelL())
                    arm.openLeftClaw();
                else if(!pixelIndicator.isTherePixelR())
                    arm.openRightClaw();
                while(!pixelIndicator.isThereAnyPixel() && timer.seconds() < 1){
                    chassis.forward(0.6);
                }
                if(pixelIndicator.isTherePixelL())
                    arm.closeLeftClaw();
                if(pixelIndicator.isTherePixelR())
                    arm.closeRightClaw();
                chassis.stop();
            }

            //claw
            if (gamepad2.right_trigger > 0) {
                arm.toggleClaw();
                delay = true;
            }
            if(gamepad2.left_bumper || gamepad2.right_bumper){
                if (gamepad2.left_bumper && arm.getLiftPosition() != Arm.GROUND) arm.toggleRightClaw();
                else if(gamepad2.left_bumper && arm.getLiftPosition() == Arm.GROUND)  arm.toggleLeftClaw();
                if (gamepad2.right_bumper && arm.getLiftPosition() != Arm.GROUND) arm.toggleLeftClaw();
                else if(gamepad2.right_bumper && arm.getLiftPosition() == Arm.GROUND) arm.toggleRightClaw();
                delay = true;
            }

            //claw flip only (wrist 2)
            if(gamepad2.right_stick_x > 0.3){
                arm.setClawFlip(!arm.getClawFlip());
                delay = true;
            }

            //claw rotator only (wrist 1)
            if(gamepad2.right_stick_x < -0.3){
                arm.toggleClawRotator();
                delay = true;
            }

            //lift (arm)
            if(gamepad2.right_stick_y > 0.3) {
                //wrist down, claw open
                droneState = 0;
                arm.hang = false;
                arm.openClaw();
                arm.setLiftPosition(Arm.GROUND);
                arm.setRotatorLevel(0);
                arm.setArmExtensionPosition(15);
                arm.setClawFlip(false);
                delay = true;
            } else if (gamepad2.right_stick_y < -0.3) {
                //wrist up, claw closed
                droneState = 0;
                arm.closeClaw();
                arm.setRotatorLevel(2);
                arm.setLiftPosition(Arm.GROUND);
                arm.setArmExtensionPosition(0);
                arm.setClawFlip(false);
                delay = true;
            } else if (gamepad2.x) {
                //front drop
                arm.hang = false;
                droneState = 0;
                if(gamepad2.dpad_down){
                    arm.setLiftPosition(Arm.AUTON);
                    arm.setArmExtensionPosition(400);
                    arm.setRotatorLevel(1);
                } else if(gamepad2.dpad_left) {
                    arm.setLiftPosition(Arm.VERY_LOW);
                    arm.setArmExtensionPosition(500);
                    arm.setRotatorLevel(1);
                } else if (gamepad2.dpad_up){
                    arm.setLiftPosition(Arm.LOW);
                    arm.setArmExtensionPosition(600);
                    arm.setRotatorLevel(1);
                } else if (gamepad2.dpad_right){
                    arm.setLiftPosition(Arm.MIDDLE);
                    arm.setArmExtensionPosition(1200);
                    arm.setRotatorLevel(5);
                }
                arm.setClawFlip(true);
            } else if(gamepad2.y && gamepad2.dpad_down){
                arm.hang = true;
                arm.setLiftPosition(Arm.HANG);
                arm.setArmExtensionPosition(0);
                arm.setClawFlip(false);
            } else if (gamepad2.b) {
                droneState = 0;
                arm.hang = false;
                arm.setLiftPosition(Arm.HIGH);
                arm.setArmExtensionPosition(0);
                arm.setRotatorLevel(3);
                arm.setClawFlip(false);
            }

            if(!gamepad2.x && !gamepad2.a && !gamepad2.y){
                //armEx
                if (gamepad2.dpad_down) arm.setArmExtensionPosition(0);
                else if (gamepad2.dpad_left) arm.setArmExtensionPosition(500);
                else if (gamepad2.dpad_up) arm.setArmExtensionPosition(1000);
                else if (gamepad2.dpad_right) arm.setArmExtensionPosition(1550);
            }

            //manual lift
            if(gamepad2.y && gamepad2.x) arm.resetLift();
            if(gamepad2.y && gamepad2.left_stick_y != 0.0) arm.setLiftPower(-0.5 * gamepad2.left_stick_y);
            else arm.update(true);
            //TODO: this is not the ideal structure bc manual lift is restricting other movements

            //drone
            if (gamepad2.y && gamepad2.dpad_up) {
                droneState++;
                delay = true;
            }

            if(droneState == 1) drone.prepareLaunch();
            else if(droneState == 2) drone.launch();
            else drone.home();

            //telemetry
            telemetry.addData("arm position", arm.getLiftPosition());
            telemetry.addData("arm target", arm.getLiftTargetPosition());
            telemetry.addLine();
            telemetry.addData("drone position", drone.positionDrone.getPosition());
            telemetry.addLine();
            telemetry.addData("left claw position", arm.getLeftClawPosition());
            telemetry.addData("right claw position", arm.getRightClawPosition());
            telemetry.addLine();
            telemetry.addData("pivot encoder", arm.getClawPivotPosition());
            telemetry.addData("rotator  encoder",arm.getRotatorPosition());
            telemetry.addLine();
            telemetry.addData("ArmEx power", arm.getArmExPower());
            telemetry.addData("ArmEx position", arm.getArmExPosition());
            telemetry.addData("ArmEx target position", arm.getArmExTargetPosition());
            telemetry.addLine();
            telemetry.addData("limit switch", arm.slideZeroReset.getValue());

            telemetry.update();

            if(delay) sleep(200);
        }
    }
}