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

@TeleOp(name="Jan 14 TeleOp")
public class MainTeleOp extends LinearOpMode {
    RevBlinkinLedDriver.BlinkinPattern pattern;

    @Override
    public void runOpMode() {
        //initialize components
        Chassis chassis = new Chassis(hardwareMap);
        double forwardSpeed, strafeSpeed, rotateSpeed;
        Arm arm = new Arm(hardwareMap);
        arm.setLiftPosition(arm.GROUND);
        Drone drone = new Drone(hardwareMap);
        RevBlinkinLedDriver ledL = hardwareMap.get(RevBlinkinLedDriver.class,"blinkinL");
        RevBlinkinLedDriver ledR = hardwareMap.get(RevBlinkinLedDriver.class,"blinkinR");
        DcMotor blinkinPower = hardwareMap.get(DcMotor.class,"blinkinPower");
        PixelIndicator pixelIndicator = new PixelIndicator(hardwareMap);

        //log data
        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        arm.setArmExtensionPosition(0);

        while (opModeIsActive()) {
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

            //light indicator when both claw is closed.
            if(arm.leftClawOpen && arm.rightClawOpen) {
                if (pixelIndicator.isTherePixelR()) {
                    blinkinPower.setPower(0.5);
                    pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                    pattern = pattern.next();
                    ledR.setPattern(pattern);
                } else if (pixelIndicator.isTherePixelL()) {
                    pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                    pattern = pattern.next();
                    ledL.setPattern(pattern);
                }
            } else {
                blinkinPower.setPower(0.0);
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
                pattern = pattern.next();
                ledR.setPattern(pattern);
                ledL.setPattern(pattern);
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
            if (gamepad2.left_trigger > 0){
                arm.openClaw();
                arm.setLiftPosition(arm.GROUND);
                arm.setRotatorLevel(0);
                arm.setArmExtensionPosition(0);
                arm.setClawFlip(false);
            } else if (gamepad2.a) {
                arm.closeClaw();
                arm.setLiftPosition(arm.GROUND);
                arm.setRotatorLevel(2);
                arm.setArmExtensionPosition(0);
                arm.setClawFlip(false);
            }
            else if (gamepad2.x) {
                if(gamepad2.dpad_down || gamepad2.dpad_left){
                    if(gamepad2.dpad_down) {
                        arm.setLiftPosition(arm.LOW);
                        arm.setArmExtensionPosition(600);
                        arm.setRotatorLevel(1);
                    } else if (gamepad2.dpad_left){
                        arm.setLiftPosition(arm.MIDDLE);
                        arm.setArmExtensionPosition(1500);
                        arm.setRotatorLevel(5);
                    }

                    arm.setClawFlip(true);
                }
            } else if (gamepad2.y) {
                arm.setLiftPosition(arm.HANG);
                arm.setClawFlip(false);
            }
            else if (gamepad2.b) {
                arm.setLiftPosition(arm.HIGH);
                arm.setArmExtensionPosition(0);
                arm.setRotatorLevel(2);
                arm.setClawFlip(false);
            }

            //armEx
            if (!gamepad2.x && gamepad2.dpad_down) arm.setArmExtensionPosition(0);
            else if (!gamepad2.x && gamepad2.dpad_left) arm.setArmExtensionPosition(1000);
            else if (gamepad2.dpad_up) arm.setArmExtensionPosition(1550);
            //claw rotator
            else if (gamepad2.dpad_right) arm.toggleClawRotator();

            //manual lift
            if(gamepad2.left_stick_button) arm.resetLift();
            if(gamepad2.left_stick_y != 0.0) arm.setLiftPower(-0.5 * gamepad2.left_stick_y);
            else if(gamepad2.right_stick_y != 0.0) arm.setArmExtensionPower(-0.5 * gamepad2.right_stick_y);
            else arm.update(true);
            //TODO: this is not the ideal structure bc manual lift is restricting other movements, but its ok

            //manual armEx
            if(gamepad2.right_stick_button) arm.resetArmExtension();
            if(gamepad2.right_stick_y != 0.0) arm.setArmExtensionPower(-0.5 * gamepad2.right_stick_y);

            //drone
            if (gamepad2.a || gamepad2.x || gamepad2.left_trigger > 0) drone.home();
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