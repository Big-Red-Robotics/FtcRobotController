package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.NewArm;

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

        double speed;
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                speed = 0.3;
            } else if (gamepad1.right_bumper) {
                speed = 1;
            } else {
                speed = 0.7;
            }
            chassis.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y * speed,
                            -gamepad1.left_stick_x * speed,
                            -gamepad1.right_stick_x * speed
                    )
            );
            chassis.update();

            if (gamepad2.left_bumper) arm.openClaw();
            if (gamepad2.right_bumper) arm.closeClaw();

            if(gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_right){
                arm.setState(NewArm.ArmState.none);
                if (gamepad2.right_trigger > 0.1) arm.setLiftPower(gamepad2.right_trigger*.75, telemetry);
                if (gamepad2.left_trigger > 0.1) arm.setLiftPower(-gamepad2.left_trigger*.5, telemetry);
            }

            if (gamepad2.a) arm.setState(NewArm.ArmState.intake);
            if (gamepad2.x) arm.setState(NewArm.ArmState.outtake);
            if (gamepad2.y) arm.setState(NewArm.ArmState.hang);
            arm.update();

            telemetry.addData("arm position", arm.getLiftPosition());
            telemetry.addData("current mode", arm.currentState);

//            telemetry.addData("wrist encoder",arm.clawRotator.getCurrentPosition());

            telemetry.update();
        }
    }
}
