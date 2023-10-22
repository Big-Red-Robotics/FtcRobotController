package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Chassis;

@TeleOp
public class ChassisTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        //initialize components
        Chassis chassis = new Chassis(hardwareMap);

        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            //move components and log data

            chassis.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            chassis.update();

            telemetry.addData("FR", chassis.motorFR.getCurrentPosition());
            telemetry.addData("FL", chassis.motorFL.getCurrentPosition());
            telemetry.addData("BR", chassis.motorBR.getCurrentPosition());
            telemetry.addData("BL", chassis.motorBL.getCurrentPosition());
            telemetry.update();
        }
    }
}