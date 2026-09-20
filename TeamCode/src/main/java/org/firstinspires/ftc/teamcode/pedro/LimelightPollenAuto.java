package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.ColorResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.List;

@Autonomous(name = "BIOBUZZ Limelight Pollen Test")
public class LimelightPollenAuto extends OpMode {

    private Follower follower;
    private Limelight3A limelight;

    private int visionCycles = 0;
    private static final int MAX_VISION_CYCLES = 5;

    // =========================================================================
    // LIMELIGHT MOUNTING OFFSETS (Relative to Robot Center of Rotation)
    // =========================================================================
    // Distance forward (+) or backward (-) from robot center in inches
    private static final double CAM_X_OFFSET = 0.0;

    // Distance left (+) or right (-) from robot center in inches
    private static final double CAM_Y_OFFSET = 5.0; // 5 inches to the LEFT

    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startPose = p.of(22.55, 116.45, 180);

    private boolean driving = false;

    @Override
    public void init() {

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);

        // ONLY ONE pipeline should be active at a time.
        // 5 = your yellow Pollen pipeline.
        limelight.pipelineSwitch(5);
        limelight.start();

        telemetry.addLine("Limelight + Pedro ready");
        telemetry.update();
    }

    @Override
    public void start() {
        visionCycles = 0;
        driving = false;

        // First scan.
        scanAndDrive();
    }

    private void scanAndDrive() {
        // Stop after the allowed number of movement decisions.
        if (visionCycles >= MAX_VISION_CYCLES) {
            return;
        }

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            telemetry.addLine("No valid Limelight result");
            return;
        }

        List<ColorResult> blobs = result.getColorResults();

        if (blobs.isEmpty()) {
            telemetry.addLine("No pollen detected");
            return;
        }

        ColorResult best = blobs.get(0);
        Pose current = follower.pose();

        /*
         * Limelight TX:
         * Positive = target is to the right of the camera center line.
         * Negative = target is to the left of the camera center line.
         *
         * Convert TX to standard angle relative to camera (CCW positive):
         */
        double txRad = Math.toRadians(-best.getTargetXDegrees());

        /*
         * Distance drive assumption (12 inches for this test).
         */
        double driveDistance = 12.0;

        /*
         * 1. Calculate Target position relative to the CAMERA (in inches)
         */
        double targetX_cam = driveDistance * Math.cos(txRad);
        double targetY_cam = driveDistance * Math.sin(txRad);

        /*
         * 2. Calculate Target position relative to the ROBOT CENTER (in inches)
         * Account for the 5-inch left offset (CAM_Y_OFFSET = +5.0)
         */
        double targetX_robot = CAM_X_OFFSET + targetX_cam;
        double targetY_robot = CAM_Y_OFFSET + targetY_cam;

        /*
         * 3. Transform Robot-Relative coordinates into FIELD coordinates
         */
        double currentHeading = current.heading(); // radians

        double targetX_field = current.x()
                + (targetX_robot * Math.cos(currentHeading) - targetY_robot * Math.sin(currentHeading));

        double targetY_field = current.y()
                + (targetX_robot * Math.sin(currentHeading) + targetY_robot * Math.cos(currentHeading));

        /*
         * 4. Compute target heading to aim the robot's center directly at the ball
         */
        double targetHeadingField = Math.atan2(
                targetY_field - current.y(),
                targetX_field - current.x()
        );

        Pose target = p.of(
                targetX_field,
                targetY_field,
                Math.toDegrees(targetHeadingField)
        );

        Path visionPath = line(current, target).constant(target);

        follower.follow(visionPath);
        driving = true;

        // This counts actual vision-driven movements, not loop iterations.
        visionCycles++;
    }

    @Override
    public void loop() {
        follower.update();

        if (driving && !follower.isBusy()) {
            driving = false;
            scanAndDrive();
        }

        if (!driving && visionCycles < MAX_VISION_CYCLES) {
            scanAndDrive();
        }

        /*
         * Telemetry
         */
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<ColorResult> blobs = result.getColorResults();

            telemetry.addData("Pollen blobs/groups seen", blobs.size());

            if (!blobs.isEmpty()) {
                ColorResult best = blobs.get(0);
                telemetry.addData("Best TX", "%.2f", best.getTargetXDegrees());
                telemetry.addData("Best TY", "%.2f", best.getTargetYDegrees());
                telemetry.addData("Best Area", "%.2f%%", best.getTargetArea());
            }
        } else {
            telemetry.addData("Limelight", "No valid result");
        }

        telemetry.addData("Vision cycles", visionCycles + " / " + MAX_VISION_CYCLES);
        telemetry.addData("Driving", driving);
        telemetry.addData("Robot X", "%.2f", follower.pose().x());
        telemetry.addData("Robot Y", "%.2f", follower.pose().y());
        telemetry.addData("Heading", "%.1f", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Pedro Busy", follower.isBusy());

        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}