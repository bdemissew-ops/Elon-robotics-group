package edu.elon.robotics.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "parking", group = "ShadowWizardMoneyGang")

public class Parking extends AutoCommon{


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


        lookForParking();

//        while (opModeIsActive()) {
//            telemetry.addData("distance", robot.distanceSensor.getDistance(DistanceUnit.CM));
//            telemetry.update();
//        }
    }

    public void openSpace(int wallDist){
        double buffer = wallDist * .15;



    }


    public void lookForParking(){
        driveToCalibrateLightSensor(SLOW_SPEED);
        sleep(1000);

        double avg = (robot.minBrightness + robot.maxBrightness)/2.0;
        double buffer = avg * 0.3;

        // look for a white line
        robot.startMove(SLOW_SPEED, 0, 0);
        while (opModeIsActive() && robot.colorSensor.alpha() < buffer+avg) {

        }
        robot.startMove(0,0,0);
        turnIMU(90, SLOW_SPEED);
        sleep(2000);
        robot.resetDriveEncoders();

        robot.startMove(SLOW_SPEED, 0, 0);
        while (opModeIsActive() && robot.colorSensor.alpha() < buffer+avg){
            System.out.println("Current dist, " + robot.distanceSensor.getDistance(DistanceUnit.CM) + " currentDist traveled, " +  robot.convertTicksToDist(robot.motorLeft.getCurrentPosition()));
        }
        robot.startMove(0,0,0);




    }







}
