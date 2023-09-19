package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Arm;

@TeleOp
public class ArmTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Arm arm = new Arm(hardwareMap);

        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            arm.runToPosition(arm.highJunction);

            telemetry.addData("arm height", arm.getCurrentPosition());
            telemetry.update();
        }
    }
}