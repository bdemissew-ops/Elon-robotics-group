package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Pattern Driving")
public class PatternDriving extends AutoCommon{

    private final double[]  DRIVE_SEQ = {150, -150,   100,    -106.42,63.6};
    private final double[] STRAFE_SEQ = {-90, 70, -250, -90};

}