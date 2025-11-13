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


    public int parkingSpaceEnd =0;
    public int smallestOpenSpace = Integer.MAX_VALUE;
    public boolean measureDist(double startingSpaceFromWall, double avg, double buffer){
        // times this shit didn't work for reasons that would require God as a witness: 15
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
        // call me Bob the builder because I gotta fix this
        if(cmDist >= LEN_ROBOT-20){
            // update: I can't fix this and it's 1 in the god dam morning I am going to bed
            int openSpaceSize = robot.motorLeft.getCurrentPosition() - startSpace;
            if (openSpaceSize < smallestOpenSpace){
                // F U past me it's now 2:30 am and I fixed it
                parkingSpaceEnd = robot.motorLeft.getCurrentPosition();
                smallestOpenSpace = openSpaceSize;
                System.out.println("openSpaceSize = " + openSpaceSize);
                return true;
            }

            else{return false;}
        }
        else{
            return false;
        }


    }


    public void returnToBase(int turnTickLoc){
        robot.startMove(-SLOW_SPEED,0,0);
        //return back to where it turned
        while(opModeIsActive() && Math.abs(robot.motorLeft.getCurrentPosition())-20 > turnTickLoc ){}

        sleep(1000);

        turnIMU(-99,SLOW_SPEED);
        sleep(1000);

        robot.startMove(-SLOW_SPEED,0,0);

        while (opModeIsActive() && (Math.abs(robot.motorLeft.getCurrentPosition()) < distFromBase + 500)){}
        robot.startMove(0,0,0);

    }
    public int distFromBase;
    public void ParkPlace(){
        setUpPark();
        double avg = (robot.minBrightness + robot.maxBrightness)/2.0;
        double buffer = avg * 0.3;
        int turnTickLoc = robot.motorLeft.getCurrentPosition();



        double wallDist = robot.distanceSensor.getDistance(DistanceUnit.CM);
        boolean isEnoughSpace = false;
        // This while loop is about as stable as the Joker with down syndrome so run again if it fails it's a little special

        while(opModeIsActive() && (robot.colorSensor.alpha() < buffer+avg)){
            robot.startMove(SLOW_SPEED,0,0);
            isEnoughSpace = measureDist(wallDist, avg, buffer);
            if (!isEnoughSpace){
                isEnoughSpace = measureDist(wallDist, avg, buffer);

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
            //If the bipolar, down syndrome, Joker of a robot ever gets to this line of code I would like to retract calling this robot a:
            // "bipolar, down syndrome, Joker of a robot"
            park();
            // update: god damit
        }
        else{
            returnToBase(turnTickLoc);
        }




        robot.startMove(SLOW_SPEED, 0, 0);




    }


    public void park(){
        double currentDistTicks = robot.motorLeft.getCurrentPosition();
        robot.startMove(-SLOW_SPEED,0,0);
        double ticksOfLength = robot.convertDistToTicks(LEN_ROBOT - 35);

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
        distFromBase = robot.motorLeft.getCurrentPosition();
        robot.resetDriveEncoders();
    }




}
