package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;

public class DriveBase {
    private DcMotor fl, fr, bl, br;

    public void init(HardwareMap hardwareMap) {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Drives the robot, using the same sign convention the pathing library uses:
     *
     *   vertical   positive = forward       (robot relative X)
     *   horizontal positive = to the LEFT   (robot relative Y, same as the odometry Y axis)
     *   pivot      positive = clockwise     (heading decreasing)
     *
     * The powers are scaled down together if any of them would exceed 1, so the robot
     * still drives in the requested direction instead of being clipped into a different one.
     */
    public void drive(double vertical, double horizontal, double pivot) {
        double flPower = vertical - horizontal + pivot;
        double frPower = vertical + horizontal - pivot;
        double blPower = vertical + horizontal + pivot;
        double brPower = vertical - horizontal - pivot;

        double max = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                              Math.max(Math.abs(blPower), Math.abs(brPower)));

        if (max > 1) {
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;
        }

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }

    public void drive(RobotPower power) {
        if (power != null) {
            drive(power.getVertical(), power.getHorizontal(), power.getPivot());
        }
    }
}
