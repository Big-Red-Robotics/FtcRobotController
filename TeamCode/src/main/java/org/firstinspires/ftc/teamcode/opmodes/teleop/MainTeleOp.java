package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Drone;
import org.firstinspires.ftc.teamcode.components.PixelIndicator;

@TeleOp(name="2023-2024 CENTERSTAGE")
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        //initialize components
        Chassis chassis = new Chassis(hardwareMap);
        double forwardSpeed, strafeSpeed, rotateSpeed;
        Arm arm = new Arm(hardwareMap);
        arm.setLiftPosition(Arm.GROUND);
        Drone drone = new Drone(hardwareMap);
        PixelIndicator pixelIndicator = new PixelIndicator(hardwareMap);
        boolean controlReady = true;
        ElapsedTime time = new ElapsedTime();

        int droneState = 0;

        arm.setClawPivotPosition(Arm.CP_DEFAULT);

        //log data
        telemetry.addData("claw pivot encoder position", arm.getClawPivotPosition());
        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        arm.setArmExtensionPosition(0);

        while (opModeIsActive()) {
            boolean delay = false;

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

            //claw
            if (gamepad1.right_bumper && controlReady) {
                arm.toggleClaw();
                delay = true;
            }
            if((gamepad1.left_trigger > 0.3 || gamepad1.right_trigger > 0.3)  && controlReady){
                //TODO
                if (gamepad1.left_trigger  > 0.3 && arm.getLiftTargetPosition() != Arm.GROUND) arm.toggleRightClaw();
                else if(gamepad1.left_trigger  > 0.3 && arm.getLiftTargetPosition() == Arm.GROUND)  arm.toggleLeftClaw();
                if (gamepad1.right_trigger  > 0.3 && arm.getLiftTargetPosition() != Arm.GROUND) arm.toggleLeftClaw();
                else if(gamepad1.right_trigger  > 0.3&& arm.getLiftTargetPosition() == Arm.GROUND) arm.toggleRightClaw();
                delay = true;
            }

            //claw rotator (wrist 1)
            if(gamepad2.left_trigger > 0.2  && controlReady){
                arm.setRotatorLevel(0);
                delay = true;
            } else if (gamepad2.right_trigger > 0.2  && controlReady) {
                arm.setRotatorLevel(2);
                delay = true;
            }

            //claw pivot
            if(gamepad2.right_bumper && controlReady) {
                arm.setClawPivotPosition(Arm.CP_HALF);
                delay = true;
            } else if (gamepad2.left_bumper) {
                if(arm.getLiftTargetPosition() == Arm.HIGH) arm.setClawPivotPosition(Arm.CP_FLIP);
                else {
                    arm.setClawPivotPosition(Arm.CP_DEFAULT);
                    arm.setRotatorLevel(0);
                }
                delay = true;
            }

            //lift
            if (gamepad2.a && controlReady) {
                droneState = 0;
                if(arm.getLiftTargetPosition()== Arm.GROUND && arm.getRotatorLevel() == 2)
                {
                    arm.hang = false;
                    arm.openClaw();
                    arm.setLiftPosition(Arm.GROUND);
                    arm.setRotatorLevel(0);
                    arm.setArmExtensionPosition(15);
                    arm.setClawPivotPosition(Arm.CP_DEFAULT);
                    telemetry.addLine("I should return un-guarded and open");
                }else{
                    // default action, lift down, rotator guarded, no flip.
                    arm.closeClaw();
                    arm.setRotatorLevel(2);
                    arm.setLiftPosition(Arm.GROUND);
                    arm.setArmExtensionPosition(0);
                    arm.setClawPivotPosition(Arm.CP_DEFAULT);
                }
                delay = true;
            } else if (gamepad2.x) {
                droneState = 0;
                arm.hang = false;
                if(gamepad2.dpad_down){
                    arm.setLiftPosition(Arm.AUTON);
                    arm.setArmExtensionPosition(400);
                    arm.setRotatorLevel(1);
                    arm.setClawPivotPosition(Arm.CP_FLIP);
                } else if(gamepad2.dpad_left) {
                    arm.setLiftPosition(Arm.VERY_LOW);
                    arm.setArmExtensionPosition(500);
                    arm.setRotatorLevel(1);
                    arm.setClawPivotPosition(Arm.CP_FLIP);
                } else if (gamepad2.dpad_up) {
                    arm.setLiftPosition(Arm.LOW);
                    arm.setArmExtensionPosition(800);
                    arm.setRotatorLevel(1);
                    arm.setClawPivotPosition(Arm.CP_FLIP);
                } else if (gamepad2.dpad_right) {
                    arm.setLiftPosition(Arm.MIDDLE);
                    arm.setArmExtensionPosition(1550);
                    arm.setRotatorLevel(0);
                    arm.setClawPivotPosition(Arm.CP_FLIP);
                }
            } else if (gamepad2.y && controlReady) {
                arm.hang = true;
                droneState++;

                if(droneState == 1) {
                    drone.prepareLaunch();
                    arm.setLiftPosition(Arm.HANG);
                    arm.setArmExtensionPosition(0);
                    arm.setClawPivotPosition(Arm.CP_DEFAULT);
                } else if(droneState == 2) drone.launch();
                else {
                    drone.home();
                    arm.setLiftPosition(Arm.GROUND);
                    arm.setArmExtensionPosition(0);
                    arm.closeClaw();

                    droneState = 0;
                }

                delay = true;
            } else if (gamepad2.b) {
                arm.hang = false;
                arm.setLiftPosition(Arm.HIGH);
                arm.setArmExtensionPosition(50);
                arm.setRotatorLevel(2);
                arm.setClawPivotPosition(Arm.CP_DEFAULT);
            }

            if(!gamepad2.x && !gamepad2.a){
                //armEx
                if (gamepad2.dpad_down) arm.setArmExtensionPosition(0);
                else if (gamepad2.dpad_left) arm.setArmExtensionPosition(500);
                else if (gamepad2.dpad_up) arm.setArmExtensionPosition(1000);
                else if (gamepad2.dpad_right) arm.setArmExtensionPosition(1550);
            }

            //manual lift
            if(gamepad2.left_stick_button) arm.resetLift();
            if(gamepad2.left_stick_y != 0.0) arm.setLiftPower(-0.5 * gamepad2.left_stick_y);
            else arm.update(true);
            //TODO: this is not the ideal structure bc manual lift is restricting other movements

            //manual armEx
            if(gamepad2.right_stick_y != 0.0) {
                if(arm.hang) arm.setLiftPower(-1.0); //TODO: maybe flip
                else arm.setArmExtensionPower(-0.5 * gamepad2.right_stick_y);
            }
            if(gamepad2.right_stick_button) arm.setArmExtensionPosition(100);

            //drone
            if (gamepad2.a || gamepad2.x || gamepad2.left_trigger > 0 || gamepad1.b) drone.home();

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

            if(delay) {
                time.reset();
                controlReady = false;
            }
            if(time.milliseconds() > 300) controlReady = true;

            if(isStopRequested()) drone.home();
        }
    }
}