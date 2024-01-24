package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.RobotConfig;

@TeleOp
public class BlinkinTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        //initialize components
        RevBlinkinLedDriver blinkinRight = hardwareMap.get(RevBlinkinLedDriver.class, RobotConfig.blinkinRight);
        RevBlinkinLedDriver blinkinLeft = hardwareMap.get(RevBlinkinLedDriver.class, RobotConfig.blinkinLeft);

        RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE;
        blinkinRight.setPattern(pattern);
        blinkinLeft.setPattern(pattern);

        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            //move components and log data
            if(gamepad1.a){
                pattern = pattern.next();
                blinkinRight.setPattern(pattern);
                blinkinLeft.setPattern(pattern);
                sleep(500);
            }

            telemetry.addData("data", null);
            telemetry.update();
        }
    }
}