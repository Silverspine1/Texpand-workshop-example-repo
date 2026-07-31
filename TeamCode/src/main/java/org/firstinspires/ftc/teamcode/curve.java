package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.RobotConfig;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;
@Autonomous
public class curve extends OpMode {
    pathsManager paths = new pathsManager(new RobotConfig(0.02, 0.004, 0.016, 0.005, 0.08, 0.004, 0.09, 0.004, 0.005,
            0.000, 0.012, 0.002, 170, 193, 270, 920));
    follower follow = new follower(new RobotConfig(0.01, 0.004,0.01, 0.009,0.06, 0.004,0.04, 0.004,0.005, 0.00,0.00012, 0.0002,181, 130, 700, 650));


    double targetHeading = 0;
    boolean pathing = false;

    Odometry odometry = new Odometry();
    GoBildaPinpointDriver pinpoint;
    DriveBase driveBase = new DriveBase();
    private final sectionBuilder[] curve = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(0, 0), new Vector2D(0, 0), new Vector2D(120, 0)),
    };

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odometry.configurePinpoint(pinpoint);

        driveBase.init(hardwareMap);
        paths.addNewPath("curve");
        paths.buildPath(curve);

    }

    @Override
    public void loop() {
        if (!pathing) {
            follow.setPath(paths.returnPath("curve"));
            pathing = true;
            targetHeading = 90;
            follow.usePathHeadings(false);
        }
        odometry.update();

        if (pathing) {
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.getHeading(), odometry.getX(), odometry.getY(), odometry.getVelX(), odometry.getVelY());
            driveBase.drive(currentPower);

        } else {
            driveBase.drive(0, 0, 0);
        }
        telemetry.addData("X (cm)", odometry.getX());
        telemetry.addData("Y (cm)", odometry.getY());

    }
}
