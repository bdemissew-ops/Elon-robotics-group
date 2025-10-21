package edu.elon.robotics.base;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.opencv.core.Mat;

import edu.elon.robotics.auto.AutoCommon;


// .isPressed() method which returns false when the button is not being pressed, and true when it is being pressed.


public class RobotHardware {
    public IMU imu;
    // drive motors
    public DcMotor motorLeft;
    public DcMotor motorRight;
    public DcMotor motorAux;
    public final double ANGLE_OVERSHOOT = 5.14833333;
    public final double TICKS_PER_ROTATION = 537.7;
    public final double WHEEL_CIRCUMFERENCE = 30.1593;

    public final double TICKS_PER_CM = TICKS_PER_ROTATION/WHEEL_CIRCUMFERENCE;

    public final double DIS_BETWEEN_WHEEL= 25.98;
    public final double TURNING_DIAMETER = 30.5;

    public final double TURNING_CIRCUMFERENCE = TURNING_DIAMETER * Math.PI;

    public int maxBrightness;
    public int minBrightness;

    //-------

    // drive motors

    // sensors
    public RevTouchSensor touchSensor;
    public ColorSensor colorSensor;


    // Math.PI/3.0 is 60 degrees converted to radians
    public final double LR_TICKS_PER_FWD_CM = TICKS_PER_CM * Math.sin(Math.PI/3.0);
    public final double LR_TICKS_PER_SIDE_CM = TICKS_PER_CM * Math.cos(Math.PI/3.0);
    public final double AUX_TICKS_PER_SIDE_CM = TICKS_PER_CM;




    public double getHeading(){
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        return robotOrientation.getYaw(AngleUnit.DEGREES);
    }

    public final KiwiDriveRatio ratio;

    public RobotHardware(HardwareMap hardwareMap, boolean isAuto) {
        // define the drive motors
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorAux = hardwareMap.dcMotor.get("motorAux");
        motorAux.setDirection(DcMotor.Direction.REVERSE);
        motorAux.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reset the drive encoders to zero
        resetDriveEncoders();

        // setup the motor ratio
        ratio = new KiwiDriveRatio(isAuto);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;

// the usb on the control hub is pointed up toward the forward
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

// set this orientation
        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);

// now initialize the IMU with this mounting orientation
// this assumes the IMU to be in a REV Control Hub is named "imu"
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

// define the current direction as 0
        imu.resetYaw();
        // external sensors
        touchSensor = hardwareMap.get(RevTouchSensor.class, "touchSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");


    }

    public void resetDriveEncoders() {
        /*
         * This code resets the encoder values back to 0 for
         * each of the three drive motors.
         */
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorAux.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorAux.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void startMove(double drive, double strafe, double turn, double speedModifier) {
        /*
         * How much power should we apply to the left,
         * right, and aux motor?
         *
         * If all 3 motors apply the same power in the
         * same direction, the robot will turn in place.
         */
        ratio.computeRatio(drive, strafe, turn); // drive, strafe, turn
        /*
         * Limit the modifier.
         */
        speedModifier = Range.clip(speedModifier, 0.0, 1.0);

        /*
         * Apply the power to the motors.
         */
        motorLeft.setPower(ratio.powerLeft * speedModifier);
        motorRight.setPower(ratio.powerRight * speedModifier);
        motorAux.setPower(ratio.powerAux * speedModifier);
    }

    public void startMove(double drive, double strafe, double turn) {
        startMove(drive, strafe, turn, 1.0);
    }

    public int convertDistToTicks(double cm){
        double ticks = cm * TICKS_PER_CM;
        int rounded =(int) Math.round(ticks);
        return rounded;
    }

    public double convertTicksToDist(int ticks) {
        return ticks / TICKS_PER_CM;
    }


    public int convertDegreesToTicks(double degrees){
        double arc_length = (degrees/360.0) * TURNING_CIRCUMFERENCE;
        return convertDistToTicks(arc_length);
    }


}
