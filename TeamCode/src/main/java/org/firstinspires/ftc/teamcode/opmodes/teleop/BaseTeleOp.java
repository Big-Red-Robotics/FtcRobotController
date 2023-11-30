package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.NewIntake;

@TeleOp(name="Base TeleOp")
public class BaseTeleOp extends LinearOpMode {
    private boolean isIntakeOn = false;
    @Override
    public void runOpMode() {
        //initialize components
        Chassis drive = new Chassis(hardwareMap);
        NewIntake newIntake = new NewIntake(hardwareMap);

        //log data
        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        //basic joystick control
        double speed = 0.7;
        while (opModeIsActive()) {

            if(gamepad1.a && isIntakeOn){
                newIntake.stop();
                isIntakeOn = false;
            }else if(gamepad1.a && !isIntakeOn){
                newIntake.run();
                isIntakeOn = true;
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y * speed,
                            -gamepad1.left_stick_x * speed,
                            -gamepad1.right_stick_x * speed
                    )
            );



            drive.update();
        }
    }
}
