package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Arm;

@TeleOp
public class SwitchTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        //initialize components

        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            //move components and log data
            Arm arm = new Arm(hardwareMap);

            telemetry.addData("limit switch", arm.slideZeroReset.getValue());

            telemetry.update();
        }
    }
}