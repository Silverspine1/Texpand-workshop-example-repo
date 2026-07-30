package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Odometry {
    /* Everything in this class is in CM, and headings are in DEGREES from 0 to 360. */

    /* Starting pose of the robot on the field, applied every time the pinpoint is configured. */
    public static final double START_X = 0;
    public static final double START_Y = 0;
    public static final double START_HEADING = 0;

    private GoBildaPinpointDriver pinpoint;

    public void configurePinpoint(GoBildaPinpointDriver pinpoint){
        this.pinpoint = pinpoint;
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        pinpoint.setOffsets(15, 100, DistanceUnit.MM);

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();

        setStartPosition(START_X, START_Y, START_HEADING);
    }

    public void update() {
        if (pinpoint != null) {
            pinpoint.update();
        }
    }

    public void setStartPosition(double x, double y, double heading) {
        if (pinpoint != null) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.CM, x, y, AngleUnit.DEGREES, normalizeHeading(heading)));
        }
    }

    public double getX() {
        return pinpoint.getPosX(DistanceUnit.CM);
    }

    public double getY() {
        return pinpoint.getPosY(DistanceUnit.CM);
    }


    public double getHeading() {
        return normalizeHeading(pinpoint.getHeading(AngleUnit.DEGREES));
    }

    public double getVelX() {
        return pinpoint.getVelX(DistanceUnit.CM);
    }

    public double getVelY() {
        return pinpoint.getVelY(DistanceUnit.CM);
    }

    private static double normalizeHeading(double degrees) {
        return (degrees % 360 + 360) % 360;
    }
}
