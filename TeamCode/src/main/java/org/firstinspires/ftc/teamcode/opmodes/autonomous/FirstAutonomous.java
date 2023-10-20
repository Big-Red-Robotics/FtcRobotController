package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.NewArm;

@Disabled
@Autonomous(name="Oct 29 Autonomous")
public class FirstAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis(hardwareMap);
        NewArm arm = new NewArm(hardwareMap);

        telemetry.addLine("waiting to start!");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            //after initialization, before starting

            sleep(20);
        }

        while(opModeIsActive()) {
            //autonomous code
        }
    }
}
