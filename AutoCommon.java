package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.elon.robotics.base.RobotHardware;

/*
 * General autonomous methods.
 */
public class AutoCommon extends LinearOpMode {
    protected RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap, true);
    }

    protected void driveForTime(double power, long milliseconds){
        robot.startMove(power,0,0);
        sleep(milliseconds);
        robot.startMove(0,0,0);
    }

    public void turnAngle(double degrees, double maxPower){
        if (degrees < 0){
            maxPower = -1* maxPower;
        } else {
            maxPower = Math.abs(maxPower);
        }

        int ticks = Math.abs(robot.convertDegreesToTicks(degrees));
        robot.resetDriveEncoders();
        robot.startMove(0, 0.0,maxPower);

        while (opModeIsActive()&& Math.abs(robot.motorLeft.getCurrentPosition()) < ticks){

        }
        robot.startMove(0.0,0.0,0.0);

    }

    protected void driveDistance(double cmForward, double cmSide, double maxPower){
        // Forward ticks
        int ticksLeftForward =  (int)(cmForward * robot.LR_TICKS_PER_FWD_CM);
        int ticksRightForward =  (int)(-1 *cmForward * robot.LR_TICKS_PER_FWD_CM);

        // left ticks
        int ticksLeftSide = (int) (cmSide * robot.LR_TICKS_PER_SIDE_CM);
        int ticksRightside = (int) (cmSide * robot.LR_TICKS_PER_SIDE_CM);
        int tickAuxSide = (int) (cmSide* robot.AUX_TICKS_PER_SIDE_CM);

        // total ticks
        int totalTicksLeft = ticksLeftForward + ticksLeftSide;
        int totalTicksRight = (ticksRightForward) + ticksRightside;

        // Get the max ticks
        int maybeMaxTicks = Math.max(Math.abs(totalTicksLeft) , Math.abs(totalTicksRight));
        int MaxTicks = Math.max(maybeMaxTicks, Math.abs(tickAuxSide));

        double distance = Math.sqrt(Math.pow(cmForward,2) + Math.pow(cmSide,2));
        double drivePower = maxPower * cmForward/distance;
        double strafePower = maxPower * cmSide/distance;

        System.out.println("POWERS: " + drivePower + "," + strafePower);

        robot.resetDriveEncoders();
        robot.startMove(drivePower, strafePower,0.0);

        if (MaxTicks == Math.abs(totalTicksLeft)){
            while (opModeIsActive()&& Math.abs(robot.motorLeft.getCurrentPosition()) < MaxTicks){}

        }else if (MaxTicks == Math.abs(totalTicksRight)){
            while (opModeIsActive()&& Math.abs(robot.motorRight.getCurrentPosition()) < MaxTicks){}
        }else{
            while (opModeIsActive() && Math.abs(robot.motorAux.getCurrentPosition()) < MaxTicks){}
        }

//        while (opModeIsActive()&& Math.abs(robot.motorLeft.getCurrentPosition()) < MaxTicks && Math.abs(robot.motorAux.getCurrentPosition()) < MaxTicks && Math.abs(robot.motorRight.getCurrentPosition())< MaxTicks){
//
//        }
        robot.startMove(0.0,0.0,0.0);
    }

    protected void turnIMU(double degrees, double power) {
        robot.imu.resetYaw();
        power = Math.abs(power);

        if (degrees > 0) {
            power = power * -1;
        }
        robot.startMove(0, 0, power);
        while (opModeIsActive() && Math.abs(robot.getHeading()) < (Math.abs(degrees) - robot.ANGLE_OVERSHOOT)) { //Activate robot

        }
        robot.startMove(0, 0, 0);
    }

    protected void driveIMU(double cm, double power){

        // Forward ticks
        int ticksLeftForward =  (int)(cm * robot.LR_TICKS_PER_FWD_CM);




        power = Math.abs(power);

        if (cm < 0) {
            power = power * -1;
        }

        robot.resetDriveEncoders();
        robot.imu.resetYaw();
        robot.startMove(power, 0.0, 0.0);

        double kP = 0.0185;

        while (opModeIsActive() && Math.abs(robot.motorLeft.getCurrentPosition()) < ticksLeftForward) {
            double error = robot.getHeading();
            double correctPower = kP * error;
            robot.motorAux.setPower(correctPower);
        }
        robot.startMove(power, 0.0, 0.0);

    }

    protected void driveUntilTouch(double power){
        robot.startMove(power, 0.0, 0.0);

        boolean touchSensor = robot.touchSensor.isPressed();
        System.out.println("touchSensor = " + touchSensor);
        while (opModeIsActive() && !touchSensor){
            touchSensor = robot.touchSensor.isPressed();
        }
        robot.startMove(0.0, 0.0, 0.0);
    }

    // rework later TODO FIXME
    protected void driveCountTouch(double power){
        driveToCalibrateLightSensor(power);
        sleep(1000);

        int count = 0;

        int check = 0;
        // move robot forward
        robot.startMove(power, 0,0);

        double avg = (robot.maxBrightness + robot.minBrightness)/2.0;
        double buffer = avg * 0.3;


        int lineWidthTicks = 0;

        boolean touchSensor = robot.touchSensor.isPressed();
        while (opModeIsActive() && !touchSensor){
            touchSensor = robot.touchSensor.isPressed();



            // collect data
            telemetry.addData("Lines seen", count + " lines seen so far");
            telemetry.update();
            System.out.println("count = " + count);
            System.out.println("robot.maxBrightness = " + robot.maxBrightness);


            //when not grey update the count but also update the prevAlpha
            if (robot.colorSensor.alpha() >= avg +buffer && check == 0) {
                count++;
                check = 1;
                lineWidthTicks = robot.motorLeft.getCurrentPosition();

                System.out.println("robot.colorSensor.alpha() = " + robot.colorSensor.alpha());
            } else if (robot.colorSensor.alpha() < avg - buffer) { // small buffer
                check = 0;

                if (lineWidthTicks > 0){
                    lineWidthTicks = robot.motorLeft.getCurrentPosition() - lineWidthTicks;
                    double lineWidth = robot.convertTicksToDist(lineWidthTicks) + Math.cos(30) + .25;//test with cos(30)
                    System.out.println("LINEWIDTH:  Line #" + count + " = " + lineWidth  + " cm");
                }
                lineWidthTicks = 0;
            }




        }
        robot.startMove(0,0,0);

        int checker = count +1;

        robot.startMove(-power, 0,0);
        while(opModeIsActive() && checker != 0){

            if (robot.colorSensor.alpha() >= avg +buffer && check == 0) {
                checker--;
                check = 1;
                System.out.println("robot.colorSensor.alpha() = " + robot.colorSensor.alpha());
            } else if (robot.colorSensor.alpha() < avg - buffer) { // small buffer
                check = 0;
                System.out.println("robot.colorSensor.alpha() = " + robot.colorSensor.alpha());
            }

        }
        robot.startMove(0,0,0);


    }

    protected void driveToCalibrateLightSensor(double power){
        int maxVal = Integer.MIN_VALUE;

        int minVal = Integer.MAX_VALUE;

        int ticks = robot.convertDistToTicks(20);

        robot.startMove(power, 0.0,0.0);

        while (opModeIsActive() && robot.motorLeft.getCurrentPosition() < ticks){
            System.out.println("robot.colorSensor.alpha() = " + robot.colorSensor.alpha());
            if (robot.colorSensor.alpha() > maxVal){
                maxVal = robot.colorSensor.alpha();
            }

            if (robot.colorSensor.alpha() < minVal){
                minVal = robot.colorSensor.alpha();
            }


        }
        System.out.println("maxVal = " + maxVal);
        System.out.println("minVal = " + minVal);
        robot.minBrightness = minVal;
        robot.maxBrightness = maxVal;

        robot.startMove(0.0, 0.0, 0.0);

    }


}
