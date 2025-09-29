package edu.elon.robotics;

import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.elon.robotics.auto.AutoCommon;
@Autonomous(name = "Encoder Drive", group = "labs")
@Configurable
public class EncoderDrive extends AutoCommon {
    private static double driveSpeed = 1.0;
    private static int numTicks = 500;
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();


        robot.resetDriveEncoders();
        robot.startMove(0.0, 0.0, driveSpeed);
        while (opModeIsActive() && Math.abs(robot.motorLeft.getCurrentPosition()) < numTicks) {
            // nothing here for now
        }
        robot.startMove(driveSpeed, 0.0, 0.0);

        System.out.println("MYDATA: "
                + robot.motorLeft.getCurrentPosition() + ","
                + robot.motorRight.getCurrentPosition() + ","
                + robot.motorAux.getCurrentPosition());
    }

}
