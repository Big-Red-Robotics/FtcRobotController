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
            if (gamepad1.a) arm.toPosition(arm.HANG, 3,false, telemetry);
            if (gamepad1.b) arm.toPosition(arm.LOW, 1,false, telemetry);

            if (gamepad1.dpad_up) arm.setLiftPower(0.7);
            if (gamepad1.dpad_down) arm.setLiftPower(-0.7);

            if (gamepad1.left_bumper) arm.setLiftPosition(arm.GROUND);
            if (gamepad1.right_bumper) arm.setLiftPosition(arm.HIGH);
            else arm.setLiftPosition(arm.getLiftPosition());

            arm.update(false);

            telemetry.addData("arm position",arm.getLiftPosition());
            telemetry.update();
        }
    }
}