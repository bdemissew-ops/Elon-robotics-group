package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

        double kP = 0.020;

        while (opModeIsActive() && Math.abs(robot.motorLeft.getCurrentPosition()) < ticksLeftForward) {
            double error = robot.getHeading();
            double correctPower = kP * error;
            robot.motorAux.setPower(correctPower);
        }
        robot.startMove(power, 0.0, 0.0);

    }
}