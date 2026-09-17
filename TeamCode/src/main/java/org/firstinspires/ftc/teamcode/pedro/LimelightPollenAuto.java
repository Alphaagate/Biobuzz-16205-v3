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
        // 0 = your yellow Pollen pipeline.
        limelight.pipelineSwitch(0);
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

        /*
         * NEW BEHAVIOR:
         * Because we enabled "Smart Target Grouping" and set "Sort Mode" to Largest
         * in the Limelight web interface, the Limelight camera natively handles clumps.
         * Index 0 is guaranteed to be our best target (single ball or clump).
         */
        ColorResult best = blobs.get(0);

        Pose current = follower.pose();

        /*
         * Limelight TX:
         * positive = target is to the right.
         *
         * We convert TX into radians and use the robot's
         * current Pedro heading to calculate the field heading.
         */
        double tx = Math.toRadians(best.getTargetXDegrees());
        double targetHeading = current.heading() - tx;

        /*
         * For the first test, only drive 12 inches toward the target.
         * Then we scan again.
         */
        double driveDistance = 12.0;

        double targetX = current.x() + driveDistance * Math.cos(targetHeading);
        double targetY = current.y() + driveDistance * Math.sin(targetHeading);

        Pose target = p.of(
                targetX,
                targetY,
                Math.toDegrees(targetHeading)
        );

        Path visionPath = line(current, target).constant(target);

        follower.follow(visionPath);
        driving = true;

        // This counts actual vision-driven movements, not loop iterations.
        visionCycles++;
    }

    @Override
    public void loop() {
        /*
         * Always update Pedro first.
         */
        follower.update();

        /*
         * If we finished the previous 12-inch path,
         * allow another vision scan.
         */
        if (driving && !follower.isBusy()) {
            driving = false;
            scanAndDrive();
        }

        /*
         * If the first scan did not get a result,
         * try again on the next loop.
         */
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

                // Grab the pre-sorted best result from Limelight
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