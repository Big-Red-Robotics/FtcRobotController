package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Arm;

@Disabled
@TeleOp
public class ClawTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Arm arm = new Arm(hardwareMap);

        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        double pos = 0.0;
        while(opModeIsActive()) {
            if (gamepad1.a) pos += 0.01;
            if (gamepad1.b) pos -= 0.01;

            arm.claw.setPosition(pos);

            telemetry.addData("arm position",arm.claw.getPosition());
            telemetry.update();
        }
    }
}