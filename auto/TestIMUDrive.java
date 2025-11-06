package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Test IMU Driving", group = "ShadowWizardMoneyGang")
public class TestIMUDrive extends AutoCommon {

    private final double Dist = 400.0;
    private final double   SLOW_SPEED = 0.3;
    private final double MEDIUM_SPEED = 0.65;
    private final double   FAST_SPEED = 1.0;
    private final long    SHORT_PAUSE = 250;
    private final long     LONG_PAUSE = 3000;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        driveToCalibrateLightSensor(MEDIUM_SPEED);

        sleep(LONG_PAUSE);

        driveToCalibrateLightSensor(-MEDIUM_SPEED);
//
//        // do slow speed turning sequence
//        telemetry.addData("Driving", Dist + " at " + SLOW_SPEED + " speed");
//        telemetry.update();
//        driveIMU(Dist, MEDIUM_SPEED);
//        if (!opModeIsActive()) return;
//        sleep(SHORT_PAUSE);
//        System.out.println("[DRIVE_IMU] requested: " + Dist + ", final heading: " + robot.getHeading());
//        System.out.println("[HEADING] Heading "+ robot.getHeading());
//
//
//        // take a break
//        sleep(LONG_PAUSE);
//
//        // do medium speed turning sequence
//        telemetry.addData("Driving", Dist + " at " + SLOW_SPEED + " speed");
//        telemetry.update();
//        driveIMU(Dist, MEDIUM_SPEED);
//        if (!opModeIsActive()) return;
//        sleep(SHORT_PAUSE);
//        System.out.println("[DRIVE_IMU] requested: " + Dist + ", final heading: " + robot.getHeading());
//        System.out.println("[HEADING] Heading "+ robot.getHeading());
//
//
//        // take a break
//        sleep(LONG_PAUSE);
//
//        // do fast speed turning sequence
//        telemetry.addData("Driving", Dist + " at " + SLOW_SPEED + " speed");
//        telemetry.update();
//        driveIMU(Dist, FAST_SPEED);
//        if (!opModeIsActive()) return;
//        sleep(SHORT_PAUSE);
//        System.out.println("[DRIVE_IMU] requested: " + Dist + ", final heading: " + robot.getHeading());
//        System.out.println("[HEADING] Heading "+ robot.getHeading());
    }

}