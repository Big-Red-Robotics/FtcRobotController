package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.Drone;

@TeleOp
public class DroneTest extends LinearOpMode {

    @Override
    public void runOpMode(){
        Chassis chassis = new Chassis(hardwareMap);
        Drone drone = new Drone(hardwareMap);

        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            double speed = 0.5;
            while (opModeIsActive()) {
                chassis.setWeightedDrivePower(
                        new Pose2d(
                                gamepad1.left_stick_y * speed,
                                -gamepad1.left_stick_x * speed,
                                -gamepad1.right_stick_x * speed
                        )
                );

                chassis.update();
            }

            if(gamepad1.a) drone.home();
            if(gamepad1.b) drone.positionDrone.setPosition(0.30);
            if(gamepad1.x) drone.positionDrone.setPosition(0.35);
            if(gamepad1.y) drone.prepareLaunch();

//            if (gamepad1.a) launchPos += 0.01;
//            if (gamepad1.b) launchPos -= 0.01;
//            drone.launchDrone.setPosition(launchPos);
//
//            if (gamepad1.dpad_up) posPos += 0.01;
//            if (gamepad1.dpad_down) posPos -= 0.01;
//            drone.positionDrone.setPosition(posPos);

//            telemetry.addData("launch",drone.launchDrone.getPosition());
            telemetry.addData("position",drone.positionDrone.getPosition());
            telemetry.update();
        }
    }
}
