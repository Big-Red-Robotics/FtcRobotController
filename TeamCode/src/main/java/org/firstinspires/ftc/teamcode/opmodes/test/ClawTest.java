package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.components.Arm;

@TeleOp
public class ClawTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Arm arm = new Arm(hardwareMap);

        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        arm.clawPivot.setPwmRange(new PwmControl.PwmRange(1200, 1800));

        while(opModeIsActive()) {
            if (gamepad1.a) arm.clawPivot.setPosition(0.29);
            if (gamepad1.b) arm.clawPivot.setPosition(0.95);

            telemetry.addData("arm position",arm.clawPivot.getPosition());
            telemetry.update();
        }
    }
}