package org.firstinspires.ftc.teamcode.opmodes.teleop;

import java.lang.String;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.*;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;

@TeleOp(name="Base TeleOp")
public class BaseTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        //initialize components
        Chassis drive = new Chassis(hardwareMap);
        NewIntake newIntake = new NewIntake(hardwareMap);
        NewArm newArm = new NewArm(hardwareMap, 0.3);
        // **COMPELX MODE MUST START AT BOTTOM**
        NewLift newLift = new NewLift(hardwareMap, false, 0.4);
        NewClaw claw = new NewClaw(hardwareMap);

        //log data
        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        //basic joystick control
        double speed = RobotConfig.joyStickSpeed;
        while (opModeIsActive()) {



            if(gamepad1.right_bumper)
                newIntake.run();
            newIntake.stop();

                //better arm.
            if(gamepad1.y)
                newArm.run();
            else if(gamepad1.a)
                newArm.back();

            newArm.stop();


            // Lift

            if(!newLift.isExceedingLimit()){
                if(gamepad2.y)
                    newLift.up();
                else if(gamepad2.a)
                    newLift.down();
            }
            newLift.stop();


            if(gamepad1.x)
                claw.open();
            if(gamepad1.y)
                claw.close();

            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y * speed,
                            -gamepad1.left_stick_x * speed,
                            -gamepad1.right_stick_x * speed
                    )
            );

            telemetry.addLine(String.valueOf(newLift.getCurrentPosition()));
            telemetry.addLine(String.valueOf(claw.getPos()));

            //TODO  BEFORE RUN, add telemtry update the life and servo positions. Also string.valuesof i don't have it yet.

            drive.update();
        }
    }
}
