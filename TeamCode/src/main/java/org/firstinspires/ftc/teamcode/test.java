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

    /* How close (in cm) to the end of the path counts as finished. */
    private static final double END_TOLERANCE = 8;

    PathsManager paths = new PathsManager(new RobotConfig()
            .setXLastAdjustmentPD(0.02, 0.004)
            .setYLastAdjustmentPD(0.02, 0.009)
            .setXOnPathPD(0.08, 0.004)
            .setYOnPathPD(0.1, 0.004)
            .setFastHeadingPD(0.01, 0.0005)
            .setSlowHeadingPD(0.012, 0.002)
            .setRobotConstants(181, 130, 700, 650));

    Follower follow = new Follower(new RobotConfig()
            .setXLastAdjustmentPD(0.02, 0.004)
            .setYLastAdjustmentPD(0.02, 0.009)
            .setXOnPathPD(0.08, 0.004)
            .setYOnPathPD(0.1, 0.004)
            .setFastHeadingPD(0.01, 0.0005)
            .setSlowHeadingPD(0.012, 0.002)
            .setRobotConstants(181, 130, 700, 650));

    double targetHeading = 0;
    boolean pathing = false;

    Odometry odometry = new Odometry();
    GoBildaPinpointDriver pinpoint;
    DriveBase driveBase = new DriveBase();

    /*
     * Path points are in CM in the field frame: +Y is the direction the robot faces at heading 0
     * (forward), +X is to its left. So this "line" path drives 100 cm FORWARD.
     */
    private final SectionBuilder[] line = new SectionBuilder[] {
            () -> paths.addPoints(new Vector2D(0, 0), new Vector2D(0, 100)),
    };
    private final SectionBuilder[] curve = new SectionBuilder[] {
            () -> paths.addPoints(new Vector2D(0, 0), new Vector2D(-30, 50), new Vector2D(-50, 100)),
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

        double x = odometry.getX();
        double y = odometry.getY();
        double heading = odometry.getHeading();

        if (gamepad1.right_bumper && !pathing) {
            follow.setPath(paths.returnPath("line"));
            pathing = true;
        } else if (gamepad1.left_bumper && !pathing) {
            follow.setPath(paths.returnPath("curve"));
            pathing = true;
        }

        telemetry.addData("X (cm)", x);
        telemetry.addData("Y (cm)", y);
        telemetry.addData("Heading (deg)", heading);
        telemetry.addData("Velocity X (cm/s)", odometry.getVelX());
        telemetry.addData("Velocity Y (cm/s)", odometry.getVelY());

        if (pathing) {
            RobotPower currentPower = follow.followPathAuto(targetHeading, heading, x, y, odometry.getVelX(), odometry.getVelY());

            /*
             * Don't use follow.isFinished(x, y) - it compares the signed error to the tolerance,
             * so a large error in the negative direction counts as "finished" and the path ends
             * early with the robot nowhere near the end point.
             */
            Vector2D endError = follow.getErrorToPointOnPath(x, y);
            boolean atEnd = Math.abs(endError.getX()) < END_TOLERANCE
                    && Math.abs(endError.getY()) < END_TOLERANCE;

            telemetry.addData("End error X (cm)", endError.getX());
            telemetry.addData("End error Y (cm)", endError.getY());
            telemetry.addData("Power vertical/horizontal/pivot",
                    "%.2f / %.2f / %.2f",
                    currentPower.getVertical(), currentPower.getHorizontal(), currentPower.getPivot());

            if (atEnd) {
                pathing = false;
                driveBase.drive(0, 0, 0);
            } else {
                driveBase.drive(currentPower);
            }
        } else {
            driveBase.drive(0, 0, 0);
        }
    }
}
