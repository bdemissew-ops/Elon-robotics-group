package edu.elon.robotics.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "parking", group = "ShadowWizardMoneyGang")

public class Parking extends AutoCommon{


    private final double Dist = 400.0;
    private final double   SLOW_SPEED = 0.15;
    private final double MEDIUM_SPEED = 0.65;
    private final double   FAST_SPEED = 1.0;
    private final long    SHORT_PAUSE = 250;
    private final long     LONG_PAUSE = 3000;

    private final int LEN_ROBOT = 45;



    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        ParkPlace();

    }




    public boolean measureDist2(double startingSpaceFromWall){
        double buffer = 15;
        robot.startMove(SLOW_SPEED,0,0);
        // while I don't have enough space
        while(opModeIsActive()  && robot.distanceSensor.getDistance(DistanceUnit.CM) < LEN_ROBOT + buffer){
            //            telemetry.addData("Currently seeing wall");
        }
        robot.startMove(0,0,0);
        sleep(1000);

        robot.startMove(SLOW_SPEED,0,0);

        int trackDistTicks=robot.motorLeft.getCurrentPosition();
        //when we start seeing the wall
        while(opModeIsActive() && robot.distanceSensor.getDistance(DistanceUnit.CM) > startingSpaceFromWall+buffer){

        }
        robot.startMove(0,0,0);
        sleep(1000);

        trackDistTicks = robot.motorLeft.getCurrentPosition() - trackDistTicks;
        double cmDist = robot.convertTicksToDist(trackDistTicks);
        return cmDist >= LEN_ROBOT-10;


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
        sleep(1000);



        turnIMU(90, SLOW_SPEED);
        sleep(1000);
        robot.startMove(0,SLOW_SPEED,0);
        while(opModeIsActive() && robot.distanceSensor.getDistance(DistanceUnit.CM) > 15){

        }

        robot.startMove(0,0,0);
        sleep(1000);
        robot.resetDriveEncoders();


        boolean isEnoughSpace = measureDist2(robot.distanceSensor.getDistance(DistanceUnit.CM));

        robot.resetDriveEncoders();
        if (isEnoughSpace){

            double currentDistTicks = robot.motorLeft.getCurrentPosition();
            robot.startMove(-SLOW_SPEED,0,0);

            double ticksOfLength = robot.convertDistToTicks(LEN_ROBOT - 25);

            while(opModeIsActive() && ticksOfLength > Math.abs(currentDistTicks - robot.motorLeft.getCurrentPosition())){

            }

            robot.startMove(0,0,0);
            robot.startMove(0,SLOW_SPEED,0);
            while(opModeIsActive() && robot.distanceSensor.getDistance(DistanceUnit.CM) > 15){}
        }
        else{
            robot.startMove(SLOW_SPEED,0,0);
            while (opModeIsActive() && robot.colorSensor.alpha() < buffer+avg){}
        }

        robot.startMove(0, 0, 0);


    }



    public int parkingSpaceEnd =0;
    public boolean measureDist(double startingSpaceFromWall, double avg, double buffer){
        robot.startMove(0,0,0);
        sleep(1000);

        int startSpace = 0;
        double Distbuffer = 45;
        robot.startMove(SLOW_SPEED,0,0);
        // if we stop seeing the wall
        while(opModeIsActive() && robot.distanceSensor.getDistance(DistanceUnit.CM) < startingSpaceFromWall+Distbuffer && robot.colorSensor.alpha() < buffer+avg){
            //            telemetry.addData("Currently seeing wall");
        }
        robot.startMove(0,0,0);
        sleep(1000);

        startSpace = robot.motorLeft.getCurrentPosition();

        robot.startMove(SLOW_SPEED,0,0);

        int trackDistTicks=robot.motorLeft.getCurrentPosition();
        //when we start seeing the wall
        while(opModeIsActive() && robot.distanceSensor.getDistance(DistanceUnit.CM) > startingSpaceFromWall+Distbuffer && robot.colorSensor.alpha() < buffer+avg){

        }
        robot.startMove(0,0,0);
        sleep(1000);

        trackDistTicks = robot.motorLeft.getCurrentPosition() - trackDistTicks;
        double cmDist = robot.convertTicksToDist(trackDistTicks);

        if(cmDist >= LEN_ROBOT-10){
            parkingSpaceEnd = robot.motorLeft.getCurrentPosition();
            return true;
        }
        else{
            return false;
        }


    }


    public void ParkPlace(){
        setUpPark();
        double avg = (robot.minBrightness + robot.maxBrightness)/2.0;
        double buffer = avg * 0.3;




        double wallDist = robot.distanceSensor.getDistance(DistanceUnit.CM);
        boolean isEnoughSpace = false;

        while(opModeIsActive() && (robot.colorSensor.alpha() < buffer+avg)){
            robot.startMove(SLOW_SPEED,0,0);
            if (!isEnoughSpace){
                isEnoughSpace = measureDist(wallDist, avg, buffer);
                System.out.println("Valid Park detected: " + isEnoughSpace);
            }
        }
        // assume that you are at the line

        if (parkingSpaceEnd != 0){
            robot.startMove(0,0,0);
            sleep(1000);

            robot.startMove(-SLOW_SPEED,0,0);
            while (opModeIsActive() && Math.abs(robot.motorLeft.getCurrentPosition()) > parkingSpaceEnd){}

            robot.startMove(0,0,0);
            sleep(1000);
            park();
        }
        else{
            robot.startMove(SLOW_SPEED, 0, 0);
        }




        robot.startMove(SLOW_SPEED, 0, 0);




    }


    public void park(){
        double currentDistTicks = robot.motorLeft.getCurrentPosition();
        robot.startMove(-SLOW_SPEED,0,0);
        double ticksOfLength = robot.convertDistToTicks(LEN_ROBOT - 30);

        while(opModeIsActive() && ticksOfLength > Math.abs(currentDistTicks - robot.motorLeft.getCurrentPosition())){}

        telemetry.addData("Car is now at end park space at ", robot.motorLeft.getCurrentPosition());
        telemetry.update();

        robot.startMove(0,0,0);
        robot.startMove(0,SLOW_SPEED,0);
        while(opModeIsActive() && robot.distanceSensor.getDistance(DistanceUnit.CM) > 15){}
    }

    public void setUpPark(){
        driveToCalibrateLightSensor(SLOW_SPEED);
        sleep(1000);

        double avg = (robot.minBrightness + robot.maxBrightness)/2.0;
        double buffer = avg * 0.3;
        // look for a white line
        robot.startMove(SLOW_SPEED, 0, 0);
        while (opModeIsActive() && robot.colorSensor.alpha() < buffer+avg) {
        }

        robot.startMove(0,0,0);
        sleep(1000);
        turnIMU(90, SLOW_SPEED);
        sleep(1000);
        robot.startMove(0,SLOW_SPEED,0);
        while(opModeIsActive() && robot.distanceSensor.getDistance(DistanceUnit.CM) > 15){

        }
        robot.startMove(0,0,0);
        sleep(1000);
        robot.resetDriveEncoders();
    }




}
