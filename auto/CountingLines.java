package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "CountingLines", group = "SHAW")


// max = 5560
// min = 718


public class CountingLines extends AutoCommon {
    public double MEDIUM_SPEED = Math.PI/10;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        driveCountTouch(MEDIUM_SPEED/2);
        robot.startMove(0,0,0);
    }
}
