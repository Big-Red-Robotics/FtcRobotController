package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.NewArm;
import org.firstinspires.ftc.teamcode.components.old.Arm;

@TeleOp(name="Oct 29 TeleOp")
public class FirstTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        //initialize components
        Chassis chassis = new Chassis(hardwareMap);
        NewArm arm = new NewArm(hardwareMap);

        MecanumDrive mecanumdrive;

        //log data
        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        double speed = 0.5;
        while (opModeIsActive()) {
            chassis.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y * speed,
                            -gamepad1.left_stick_x * speed,
                            -gamepad1.right_stick_x * speed
                    )
            );
            chassis.update();

            if (gamepad1.a) arm.openClaw();
            if (gamepad1.b) arm.closeClaw();

            if(gamepad1.dpad_up || gamepad1.dpad_down){
                arm.setState(NewArm.ArmState.none);
                if (gamepad1.dpad_up) arm.setLiftPower(0.7, telemetry);
                if (gamepad1.dpad_down) arm.setLiftPower(-0.7, telemetry);
                else arm.setLiftPower(0.0, telemetry);
            }

            if (gamepad1.left_bumper) arm.setState(NewArm.ArmState.intake);
            if (gamepad1.right_bumper) arm.setState(NewArm.ArmState.outtake);
            arm.update(telemetry);

            telemetry.addData("arm position",arm.getLiftPosition());

            telemetry.addData("wrist encoder",arm.clawRotator.getCurrentPosition());

            telemetry.update();
        }
    }
}
