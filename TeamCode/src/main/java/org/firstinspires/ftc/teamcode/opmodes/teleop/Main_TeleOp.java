package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Chassis;

@TeleOp(name="Main TeleOp :)", group="Linear Opmode")
public class Main_TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        // init chassis
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        Chassis chassis = new Chassis(motorFL, motorFR, motorBL, motorBR);
        chassis.init();

        // init arms
        DcMotor leftLift = hardwareMap.get(DcMotor.class, "leftArm");
        DcMotor rightLift = hardwareMap.get(DcMotor.class, "rightArm");
        Servo gripper = hardwareMap.get(Servo.class, "gripper");
        DcMotor cam = hardwareMap.get(DcMotor.class, "cam");
        Arm arm = new Arm(leftLift, rightLift, gripper, cam);
        arm.init();
        arm.armTarget = 0;

        int camLevel = 1;

        boolean stack = false;

        waitForStart();

        while (opModeIsActive()) {
            chassis.joyStick(gamepad1);

            if (gamepad2.x) { arm.armTarget = arm.lowJunction; stack = false; }
            if (gamepad2.b) { arm.armTarget = arm.middleJunction; stack = false; }
            if (gamepad2.y) { arm.armTarget = arm.highJunction; stack = false; }
            if (gamepad2.a) { arm.armTarget = 0; camLevel = 0; stack = false; }

            if (gamepad2.dpad_up) { arm.armTarget = 350; camLevel = -565; stack = true; }
            if (gamepad2.dpad_left) { arm.armTarget = 340; camLevel = -640; stack = true; }
            if (gamepad2.dpad_right) { arm.armTarget = 325; camLevel = -705; stack = true; }
            if (gamepad2.dpad_down) { arm.armTarget = 310; camLevel = -785; stack = true; }

            if (gamepad2.left_bumper){
                arm.armTarget = 0;
                if (arm.getCurrentPosition() > arm.lowJunction) camLevel = 0;
                stack = false;
                arm.openGripper();
            }
            if (gamepad2.right_bumper) arm.closeGripper();


            //TODO: SET 1.0 FOR 11166-RC!!
            if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
                arm.armTriggers(gamepad2, camLevel);
            } else if (Math.abs(gamepad2.left_stick_y) > 0) {
                cam.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.cam.setPower(gamepad2.left_stick_y);
                camLevel = arm.cam.getCurrentPosition();
            } else if (gamepad2.right_stick_button) {
                cam.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                camLevel = 0;
            } else {
                cam.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setArmPower(gamepad2, 1.0, camLevel, stack);
            }

            telemetry.addData("arm position", arm.getCurrentPosition());
            telemetry.addData("arm target", arm.armTarget);
            telemetry.addData("Servo position", arm.gripper.getPosition());
            telemetry.addData("Cam:",arm.cam.getCurrentPosition());
            telemetry.update();
        }
    }
}
