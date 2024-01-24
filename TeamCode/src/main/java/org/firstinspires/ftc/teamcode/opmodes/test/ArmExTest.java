package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.Arm;

@TeleOp
public class ArmExTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Arm arm = new Arm(hardwareMap);

        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.a) arm.armExtension.setTargetPosition(0);
            if (gamepad1.b) arm.armExtension.setTargetPosition(800);
            if (gamepad1.y) arm.armExtension.setTargetPosition(1600);
            else arm.armExtension.setTargetPosition(arm.armExtension.getCurrentPosition());

            arm.armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(arm.armExtension.isBusy()) arm.armExtension.setPower(0.6);

            telemetry.addData("arm position",arm.armExtension.getCurrentPosition());
            telemetry.update();
        }
    }
}