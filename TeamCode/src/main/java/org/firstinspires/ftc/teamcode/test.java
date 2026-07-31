package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.weaponboy.nexus_pathing.Follower.Follower;
import dev.weaponboy.nexus_pathing.PathGeneration.PathsManager;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.SectionBuilder;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.RobotConfig;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@TeleOp(name = "test", group = "test")
public class test extends OpMode {



    PathsManager paths = new PathsManager(new RobotConfig()
            .setXLastAdjustmentPD(0.01, 0.004)
            .setYLastAdjustmentPD(0.01, 0.009)
            .setXOnPathPD(0.06, 0.004)
            .setYOnPathPD(0.04, 0.004)
            .setFastHeadingPD(0.005, 0.005)
            .setSlowHeadingPD(0.012, 0.002)
            .setRobotConstants(181, 130, 700, 650));

    Follower follow = new Follower(new RobotConfig()
            .setXLastAdjustmentPD(0.01, 0.004)
            .setYLastAdjustmentPD(0.01, 0.009)
            .setXOnPathPD(0.06, 0.004)
            .setYOnPathPD(0.04, 0.004)
            .setFastHeadingPD(0.005, 0.005)
            .setSlowHeadingPD(0.012, 0.002)
            .setRobotConstants(181, 130, 700, 650));

    double targetHeading = 0;
    boolean pathing = false;

    Odometry odometry = new Odometry();
    GoBildaPinpointDriver pinpoint;
    DriveBase driveBase = new DriveBase();


    private final SectionBuilder[] line = new SectionBuilder[] {
            () -> paths.addPoints(new Vector2D(0, 0), new Vector2D(100, 0)),
    };
    private final SectionBuilder[] curve = new SectionBuilder[] {
            () -> paths.addPoints(new Vector2D(0, 0), new Vector2D(50, -25), new Vector2D(100, -50)),
    };


    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odometry.configurePinpoint(pinpoint);

        driveBase.init(hardwareMap);

        paths.addNewPath("line");
        paths.buildPath(line);
        paths.addNewPath("curve");
        paths.buildPath(curve);
    }

    @Override
    public void loop() {
        odometry.update();


        if (gamepad1.right_bumper && !pathing) {
            follow.setPath(paths.returnPath("line"));
            pathing = true;
            targetHeading = 0;
        } else if (gamepad1.left_bumper && !pathing) {
            follow.setPath(paths.returnPath("curve"));
            pathing = true;
            targetHeading = 0;
        } else if (gamepad1.dpad_up && !pathing) {
            follow.setPath(paths.returnPath("curve"));
            follow.usePathHeadings(true);
            follow.setHeadingLookAheadDistance(100);
            follow.setHeadingOffset(-90);
            pathing = true;
        }else if (pathing && follow.isFinished(8,8)) {
            pathing = false;

        }
        telemetry.addData("X (cm)", odometry.getX());
        telemetry.addData("Y (cm)", odometry.getY());
        telemetry.addData("Heading (deg)", odometry.getHeading());
        telemetry.addData("Velocity X (cm/s)", odometry.getVelX());
        telemetry.addData("Velocity Y (cm/s)", odometry.getVelY());

        if (pathing) {
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.getHeading(), odometry.getX(), odometry.getY(), odometry.getVelX(), odometry.getVelY());
            driveBase.drive(currentPower);

        } else {
            driveBase.drive(0, 0, 0);
        }
    }
}
