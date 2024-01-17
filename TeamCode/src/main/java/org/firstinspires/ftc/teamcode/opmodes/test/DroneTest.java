package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Drone;

@TeleOp
public class DroneTest extends LinearOpMode {

    @Override
    public void runOpMode(){
        Drone drone = new Drone(hardwareMap);

        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        double launchPos = 0.0;
        double posPos = 0.0;
        while(opModeIsActive()) {
            if(gamepad1.a) drone.home();
            if(gamepad1.b) drone.prepareLaunch();
            if(gamepad1.y) drone.launch();

//            if (gamepad1.a) launchPos += 0.01;
//            if (gamepad1.b) launchPos -= 0.01;
//            drone.launchDrone.setPosition(launchPos);
//
//            if (gamepad1.dpad_up) posPos += 0.01;
//            if (gamepad1.dpad_down) posPos -= 0.01;
//            drone.positionDrone.setPosition(posPos);

            telemetry.addData("launch",drone.launchDrone.getPosition());
            telemetry.addData("position",drone.positionDrone.getPosition());
            telemetry.update();
        }
    }
}
