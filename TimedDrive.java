package edu.elon.robotics;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.elon.robotics.auto.AutoCommon;

@Autonomous(name="Timed Drive", group = "labs")
public class TimedDrive extends AutoCommon {
    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();
        waitForStart();
        driveForTime(1.0, 2500);
    }
}
