package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.NewArm;
import org.firstinspires.ftc.teamcode.components.NewIntake;

@TeleOp(name="Base TeleOp")
public class BaseTeleOp extends LinearOpMode {
    private boolean isIntakeOn = false;
    private boolean isNewArmMoving = false;
    @Override
    public void runOpMode() {
        //initialize components
        Chassis drive = new Chassis(hardwareMap);
        NewIntake newIntake = new NewIntake(hardwareMap);
        NewArm newArm = new NewArm(hardwareMap, 0.3);

        //log data
        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        //basic joystick control
        double speed = 0.7;
        while (opModeIsActive()) {

            if(gamepad1.right_bumper && isIntakeOn){
                newIntake.stop();
                isIntakeOn = false;
            }else if(gamepad1.right_bumper && !isIntakeOn){
                newIntake.run();
                isIntakeOn = true;
            }


            if(gamepad1.y && isNewArmMoving){
                newArm.stop();
                isNewArmMoving = false;
            }else if(gamepad1.y && !isNewArmMoving){
                newArm.run();
                isNewArmMoving = true;
            }

            if(gamepad1.a && isNewArmMoving){
                newArm.stop();
                isNewArmMoving = false;
            }else if(gamepad1.a && !isNewArmMoving){
                newArm.back();
                isNewArmMoving = true;
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
