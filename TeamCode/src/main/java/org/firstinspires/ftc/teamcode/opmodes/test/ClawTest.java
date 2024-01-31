package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;

@TeleOp
public class ClawTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        ServoImplEx clawPivot = hardwareMap.get(ServoImplEx.class, RobotConfig.clawPivot);

        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        clawPivot.setPwmRange(new PwmControl.PwmRange(1200, 1800));

        while(opModeIsActive()) {
            if (gamepad1.a) clawPivot.setPosition(0.29);
            if (gamepad1.b) clawPivot.setPosition(0.95);

            telemetry.addData("arm position", clawPivot.getPosition());
            telemetry.update();
        }
    }
}