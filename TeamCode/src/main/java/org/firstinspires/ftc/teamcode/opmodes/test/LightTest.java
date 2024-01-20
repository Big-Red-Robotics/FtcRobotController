package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utility.RobotConfig;

@TeleOp
public class LightTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        //initialize components
        DcMotor light = hardwareMap.get(DcMotor.class, "light");

        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {

            if(gamepad1.a) light.setPower(-0.5);

            telemetry.addData("light power", light.getPower());
            telemetry.update();
        }
    }
}