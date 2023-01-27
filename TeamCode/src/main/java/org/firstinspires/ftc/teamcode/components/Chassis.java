package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Chassis {
    public DcMotor motorFL;
    public DcMotor motorFR;
    public DcMotor motorBL;
    public DcMotor motorBR;

    private ElapsedTime runtime = new ElapsedTime();

    public Chassis(DcMotor mFL, DcMotor mFR, DcMotor mBL, DcMotor mBR){
        this.motorFL = mFL;
        this.motorFR = mFR;
        this.motorBL = mBL;
        this.motorBR = mBR;
    }

    public void init() {
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.REVERSE);

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void joyStick(Gamepad gamepad) {
        double left_y = gamepad.left_stick_y;
        double left_x = gamepad.left_stick_x;
        double strafe_side = gamepad.right_stick_x;

        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

        if (Math.abs(Math.atan2(Math.abs(left_y), Math.abs(left_x))) < Math.PI/14.0 && strafe_side == 0 && left_x != 0) {
            if (gamepad.right_bumper) {
                leftFrontPower = -1;
                rightFrontPower = 1;
                leftBackPower = 1;
                rightBackPower = -1;
            } else {
                leftFrontPower = -left_x * 0.8 - strafe_side * 0.6;
                rightFrontPower = left_x * 0.8 + strafe_side * 0.6;
                leftBackPower = left_x * 0.8 - strafe_side * 0.6;
                rightBackPower = -left_x * 0.8 + strafe_side * 0.6;
            }
        } else if (Math.abs(Math.atan2(Math.abs(left_x), Math.abs(left_y))) < Math.PI/14.0 && strafe_side == 0 && left_y != 0) {
             if (gamepad.right_bumper) {
                 leftFrontPower = 1;
                 rightFrontPower = 1;
                 leftBackPower = 1;
                 rightBackPower = 1;
             } else {
                 leftFrontPower = (left_y - left_x) * 0.7 - strafe_side * 0.7;
                 rightFrontPower = (left_y + left_x) * 0.7 + strafe_side * 0.7;
                 leftBackPower = (left_y + left_x) * 0.7 - strafe_side * 0.7;
                 rightBackPower = (left_y - left_x) * 0.7 + strafe_side * 0.7;
             }
        } else {
            leftFrontPower = (left_y - left_x) * 0.7 - strafe_side * 0.7;
            rightFrontPower = (left_y + left_x) * 0.7 + strafe_side * 0.7;
            leftBackPower = (left_y + left_x) * 0.7 - strafe_side * 0.7;
            rightBackPower = (left_y - left_x) * 0.7 + strafe_side * 0.7;
        }

        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        if (gamepad.left_bumper) {
            motorFL.setPower(leftFrontPower * 0.35);
            motorFR.setPower(rightFrontPower * 0.35);
            motorBL.setPower(leftBackPower * 0.35);
            motorBR.setPower(rightBackPower * 0.35);
        } else {
            motorFL.setPower(leftFrontPower);
            motorFR.setPower(rightFrontPower);
            motorBL.setPower(leftBackPower);
            motorBR.setPower(rightBackPower);
        }
    }

    public void runToPosition(int FL, int FR, int BL, int BR) {
        runtime.reset();

        motorFL.setTargetPosition(FL);
        motorFR.setTargetPosition(FR);
        motorBL.setTargetPosition(BL);
        motorBR.setTargetPosition(BR);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(motorFL.isBusy() || motorFR.isBusy() || motorBL.isBusy() || motorBR.isBusy()){
            if (Math.abs(motorFL.getCurrentPosition() - FL) < 350 || Math.abs(motorFR.getCurrentPosition() - FR) < 350 || Math.abs(motorBL.getCurrentPosition() - BL) < 350 || Math.abs(motorBR.getCurrentPosition() - BR) < 350) {
                motorFL.setPower(0.15);
                motorFR.setPower(0.15);
                motorBL.setPower(0.15);
                motorBR.setPower(0.15);
            } else {
                motorFL.setPower(0.5/(1+Math.pow(3,-runtime.seconds())));
                motorFR.setPower(0.5/(1+Math.pow(3,-runtime.seconds())));
                motorBL.setPower(0.5/(1+Math.pow(3,-runtime.seconds())));
                motorBR.setPower(0.5/(1+Math.pow(3,-runtime.seconds())));
            }
        }
    }

    public void forward(double power) {
        motorFL.setPower(power);
        motorFR.setPower(power);
        motorBL.setPower(power);
        motorBR.setPower(power);
    }

    public void strafe(double power) {
        motorFL.setPower(power);
        motorFR.setPower(-power);
        motorBL.setPower(-power);
        motorBR.setPower(power);
    }

    public void turn(double power) {
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setPower(power);
        motorFR.setPower(-power);
        motorBL.setPower(power);
        motorBR.setPower(-power);
    }

    public void stop() {
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    public void resetEncoder() {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
