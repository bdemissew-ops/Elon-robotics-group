package edu.elon.robotics.auto;

import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Line Follower", group = "ShadowWizardMoneyGangWithCastingSpells")
@Configurable
public class LineFollower extends AutoCommon {
    public static double  speed = 0.15;
    private final double MEDIUM_SPEED = 0.65;
    private final double   FAST_SPEED = 1.0;
    private final long    SHORT_PAUSE = 250;
    private final long     LONG_PAUSE = 3000;
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        PIDController();
        telemetry.addData("maxVal = ", robot.maxBrightness);
        telemetry.addData("minVal = ", robot.minBrightness);
        telemetry.update();



        if (!opModeIsActive()){return;}
        sleep(SHORT_PAUSE);
        }

    // highest value aprox: 6000
    // lowest value aprox:700

    public static double kpValue = 0.00006;
    public static double ki = 0.0000118285714286;
    public static double kd = 0.0000760869565217;
    public static double Pc = 0.3043478260869565;

    protected void PIDController(){
        calibrateSensor(speed);
        sleep(1000);

        robot.imu.resetYaw();
        robot.resetDriveEncoders();
        robot.startMove(speed, 0,0);
        double target = (robot.maxBrightness + robot.minBrightness) /2.0;
        double errorSum = 0;
        double prevError = 0;


        while (opModeIsActive()){
            double currentAlpha = robot.colorSensor.alpha();
            double error = target - currentAlpha;
            errorSum = 0.8 * errorSum + error;
            double turn  = (kpValue * error) + (ki *errorSum) + (kd * (error - prevError));
            robot.startMove(speed, 0, -0.5 *turn);
            prevError = error;

        }

    }

    protected void pController(){
        calibrateSensor(speed);
        sleep(1000);

//        double KpValue = (1.0/((robot.maxBrightness - robot.minBrightness)/2.0))*0.2;
        double KcValue = 0.0001;
        double KpValue = 0.6 * KcValue;
        KpValue = 0.00006;
        double Pc = 0.3043478260869565;
        double Ki = 2 * KpValue * 0.03 / Pc;
        Ki = 0.0000118285714286;
        double kd = KpValue * Pc / 8*0.03;
        kd = 0.0000760869565217;
        double target = (robot.maxBrightness + robot.minBrightness) /2.0;
        robot.startMove(speed, 0,0);

        System.out.println("README: maxBrightness = " + robot.maxBrightness);
        System.out.println("README: minBrightness = " + robot.minBrightness);
        System.out.println("README: KpValue = " + KpValue);
        System.out.println("README: target = " + target);


        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive()){
            System.out.println("KPControl: " + timer.milliseconds() + "," + robot.colorSensor.alpha());


            double error = target - robot.colorSensor.alpha();
            double corrected_power = KpValue *error;

            System.out.println("README: error = " + error);
            System.out.println("README: corrected_power = " + corrected_power);

            robot.startMove(speed, 0, corrected_power);
        }




    }

    protected void calibrateSensor(double power){
            double[]  CALIBRATION_ANGLE_SEQ = {45,-90,45};
            for(double degrees : CALIBRATION_ANGLE_SEQ) {
                robot.imu.resetYaw();
                if (degrees > 0) {
                    power = power * -1;

                }
                else{power = Math.abs(power);}
                System.out.println("degrees = " + degrees);

                int maxVal = Integer.MIN_VALUE;

                int minVal = Integer.MAX_VALUE;

                robot.startMove(0, 0, power);
                while (opModeIsActive() && Math.abs(robot.getHeading()) < (Math.abs(degrees) - robot.ANGLE_OVERSHOOT)) { //Activate robot
                    if (robot.colorSensor.alpha() > maxVal) {
                        maxVal = robot.colorSensor.alpha();
                    }

                    if (robot.colorSensor.alpha() < minVal) {
                        minVal = robot.colorSensor.alpha();
                    }
                }
                robot.startMove(0, 0, 0);

                robot.minBrightness = minVal;
                robot.maxBrightness = maxVal;
            }
    }
}