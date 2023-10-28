package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.old.Vision;

@Disabled
@TeleOp
public class PoleTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis(hardwareMap);
        Vision vision = new Vision(hardwareMap);

        telemetry.addLine("waiting to start!");
        telemetry.update();
        waitForStart();

        vision.setDetector("pole");

        while(opModeIsActive()) {
            while (Math.abs(vision.getAutonPipeline().differenceX()) > 15) {
                double direction = Math.abs(vision.getAutonPipeline().differenceX())/vision.getAutonPipeline().differenceX();
                double power = (Math.abs(vision.getAutonPipeline().differenceX()) > 50) ? 0.3 * direction : 0.1 * direction;
                chassis.rotate(power);
                telemetry.addData("difference", vision.getAutonPipeline().differenceX());
                telemetry.update();
            }
            chassis.stop();
        }
    }
}