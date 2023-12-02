package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.NewArm;

@TeleOp
public class ArmTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        NewArm arm = new NewArm(hardwareMap);

        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.a) arm.toPosition(NewArm.ArmState.hang);
            if (gamepad1.b) arm.toPosition(NewArm.ArmState.level1);

            if (gamepad1.dpad_up) arm.setLiftPower(0.7);
            if (gamepad1.dpad_down) arm.setLiftPower(-0.7);

            if (gamepad1.left_bumper) arm.setState(NewArm.ArmState.intake);
            if (gamepad1.right_bumper) arm.setState(NewArm.ArmState.outtake);
            else arm.setState(NewArm.ArmState.none);

            arm.update();

            telemetry.addData("arm position",arm.getLiftPosition());
            telemetry.update();
        }
    }
}