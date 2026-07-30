package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Direction Test", group = "test")
public class DirectionTest extends OpMode {

    private static final double POWER = 0.3;

    Odometry odometry = new Odometry();
    GoBildaPinpointDriver pinpoint;
    DriveBase driveBase = new DriveBase();

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odometry.configurePinpoint(pinpoint);
        driveBase.init(hardwareMap);
        odometry.setStartPosition(20,5,5);

    }

    @Override
    public void loop() {
        odometry.update();

        double vertical = 0;
        double horizontal = 0;
        double pivot = 0;
        String expected = "-";

        if (gamepad1.dpad_up) {
            vertical = POWER;
            expected = "drive FORWARD, Y increases";
        } else if (gamepad1.dpad_down) {
            vertical = -POWER;
            expected = "drive BACKWARD, Y decreases";
        } else if (gamepad1.dpad_left) {
            horizontal = -POWER;
            expected = "strafe LEFT, X decreases";
        } else if (gamepad1.dpad_right) {
            horizontal = POWER;
            expected = "strafe RIGHT, X  increases";
        } else if (gamepad1.right_bumper) {
            pivot = POWER;
            expected = "turn CLOCKWISE, heading decreases";
        } else if (gamepad1.left_bumper) {
            pivot = -POWER;
            expected = "turn ANTICLOCKWISE, heading increases";
        }

        driveBase.drive(vertical, horizontal, pivot);

        telemetry.addData("Expected", expected);
        telemetry.addData("X (cm)", odometry.getX());
        telemetry.addData("Y (cm)", odometry.getY());
        telemetry.addData("Heading (deg)", odometry.getHeading());
        telemetry.addData("Velocity X (cm/s)", odometry.getVelX());
        telemetry.addData("Velocity Y (cm/s)", odometry.getVelY());
    }
}
