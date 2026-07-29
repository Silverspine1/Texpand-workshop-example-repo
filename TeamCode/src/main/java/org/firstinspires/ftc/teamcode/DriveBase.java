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
    }

    public void drive(double vertical, double horizontal, double pivot) {
        fl.setPower(vertical + horizontal + pivot);
        fr.setPower(vertical - horizontal - pivot);
        bl.setPower(vertical - horizontal + pivot);
        br.setPower(vertical + horizontal - pivot);
    }

    public void drive(RobotPower power) {
        if (power != null) {
            drive(power.getVertical(), power.getHorizontal(), power.getPivot());
        }
    }
}
