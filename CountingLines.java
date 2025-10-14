package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "CountingLines", group = "ShadowWizardMoneyGang")


// max = 5560
// min = 718


public class CountingLines extends AutoCommon {
    public double MEDIUM_SPEED = 0.3145;
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        driveToCalibrateLightSensor(MEDIUM_SPEED);

        int ticks = robot.convertDistToTicks(140);
        //make a count for the gray
        int count = 1;

        int check = 0;
        // move robot forward
        robot.startMove(MEDIUM_SPEED, 0,0);

        double avg = (robot.maxBrightness + robot.minBrightness)/2.0;
        double buffer = avg * 0.3;

        while (opModeIsActive() && robot.motorLeft.getCurrentPosition() < ticks){
            // collect data
            telemetry.addData("Lines seen", count + " lines seen so far");
            System.out.println("count = " + count);
            System.out.println("robot.maxBrightness = " + robot.maxBrightness);


            //when not grey update the count but also update the prevAlpha
            if (robot.colorSensor.alpha() >= avg +buffer && check == 0) {
                count++;
                check = 1;
                System.out.println("robot.colorSensor.alpha() = " + robot.colorSensor.alpha());
            } else if (robot.colorSensor.alpha() < avg - buffer) { // small buffer
                check = 0;
                System.out.println("robot.colorSensor.alpha() = " + robot.colorSensor.alpha());
            }
        }
        robot.startMove(0,0,0);
    }
}
