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

        double forwardSpeed;
        double strafeSpeed;
        double rotateSpeed;
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                forwardSpeed = 0.35;
                strafeSpeed = 0.25;
                rotateSpeed = 0.35;
            } else if (gamepad1.right_bumper) {
                forwardSpeed = 1;
                strafeSpeed = 0;
                rotateSpeed = 1;
            } else {
                forwardSpeed = 0.65;
                strafeSpeed = 0.5;
                rotateSpeed = 0.65;
            }
            chassis.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y * forwardSpeed,
                            -gamepad1.left_stick_x * strafeSpeed,
                            -gamepad1.right_stick_x * rotateSpeed
                    )
            );
            chassis.update();

            if (gamepad2.left_bumper) arm.openClaw();
            if (gamepad2.right_bumper) arm.closeClaw();

            if(gamepad2.right_trigger > .01 || gamepad2.left_trigger > .01){
                arm.setState(NewArm.ArmState.none);
                if (gamepad2.right_trigger > 0.01) arm.setLiftPower(gamepad2.right_trigger*.75, telemetry);
                if (gamepad2.left_trigger > 0.01) arm.setLiftPower(-gamepad2.left_trigger*.5, telemetry);
            }

            if (gamepad2.a) arm.setState(NewArm.ArmState.intake);
            if (gamepad2.x) arm.setState(NewArm.ArmState.outtake);
            if (gamepad2.y) arm.setState(NewArm.ArmState.hang);
            arm.update();

            telemetry.addData("arm position", arm.getLiftPosition());
            telemetry.addData("current mode", arm.currentState);

            telemetry.addData("wrist encoder",arm.clawRotator.getPosition());

            telemetry.update();
        }
    }
}
