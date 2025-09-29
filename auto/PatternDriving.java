package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Pattern Driving", group = "ShadowWizardMoneyGang")
public class PatternDriving extends AutoCommon {

    private final double[] DRIVE_SEQ = {150, -50,-90, 100,70, -106.42,-250, 63.6,-90};
    private final double SLOW_SPEED = 0.3;

    //private final double[] STRAFE_SEQ = {-90, 70, -250, -90};

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        for (int i = 0; i < DRIVE_SEQ.length; i++) {
            if (i == 0){
                driveDistance(DRIVE_SEQ[i], 0, SLOW_SPEED);
                if (!opModeIsActive()) return;
                sleep(250);
            } else if (i % 2 != 0){
                driveDistance(DRIVE_SEQ[i], 0, SLOW_SPEED);
                if (!opModeIsActive()) return;
                sleep(250);
            } else{
                turnAngle(DRIVE_SEQ[i],SLOW_SPEED);
            }

        }
    }
}