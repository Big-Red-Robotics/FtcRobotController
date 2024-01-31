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
            if (gamepad1.a) arm.setArmExtensionPosition(0);
            if (gamepad1.b) arm.setArmExtensionPosition(800);
            if (gamepad1.y) arm.setArmExtensionPosition(1600);
            else arm.setArmExtensionPosition(arm.getArmExPosition());

//            arm.armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            while(arm.armExtension.isBusy()) arm.armExtension.setPower(0.6);

            telemetry.addData("arm position",arm.getArmExPosition());
            telemetry.update();
        }
    }
}