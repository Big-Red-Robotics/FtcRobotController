package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Chassis;

@Disabled
@Autonomous(name="Base Autonomous")
public class _BaseAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() {
        //initialize components
        Chassis chassis = new Chassis(hardwareMap);

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
