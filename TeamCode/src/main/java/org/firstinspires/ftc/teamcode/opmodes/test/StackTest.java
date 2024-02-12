package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.PixelIndicator;

@TeleOp
public class StackTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Arm arm = new Arm(hardwareMap);
        Chassis chassis = new Chassis(hardwareMap);

        telemetry.addLine("waiting to start!");
        telemetry.update();
        PixelIndicator pixelIndicator = new PixelIndicator(hardwareMap);
        waitForStart();

        int liftpos = 110;
        double clawpos = 0.0;

        while(opModeIsActive()) {
            // lifting the arm + extending?
            sleep(200);
            if(gamepad1.dpad_up)
                liftpos += 10;
            if(gamepad1.dpad_down)
                liftpos -= 10;
            if(gamepad1.dpad_right)
                clawpos -= 0.02;
            if(gamepad1.dpad_left)
                clawpos += 0.02;

            arm.setLiftPosition(liftpos);
            arm.setClawRotatorPosition(clawpos);

            if(gamepad1.a){
                ElapsedTime timer = new ElapsedTime();
                timer.reset();
                if(!pixelIndicator.isTherePixelL())
                    arm.openLeftClaw();
                while(!pixelIndicator.isThereAnyPixel() && timer.seconds() < 1){
                    chassis.forward(0.2);
                }

                if(pixelIndicator.isTherePixelL())
                    arm.closeLeftClaw();

                chassis.stop();
            }


            if(gamepad1.b) {
                arm.toggleClaw();
            }


            arm.update(false);



//            arm.armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            while(arm.armExtension.isBusy()) arm.armExtension.setPower(0.6);

            telemetry.addData("arm position",arm.getLiftPosition());
            telemetry.addData("Desired lift",liftpos);
            telemetry.addData("claw pos", clawpos);
            telemetry.addData("arm ex", arm.getArmExPosition());
            telemetry.update();
        }
    }
}