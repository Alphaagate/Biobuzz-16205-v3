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

    private final Pose startPose =
            p.of(22.55, 116.45, 180);

    private boolean driving = false;

    @Override
    public void init() {

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        limelight =
                hardwareMap.get(
                        Limelight3A.class,
                        "limelight"
                );

        limelight.setPollRateHz(100);

        // ONLY ONE pipeline should be active at a time.
        // 0 = your yellow Pollen pipeline.
        // Do NOT switch to 1 and 2 here.
        limelight.pipelineSwitch(0);

        limelight.start();

        telemetry.addLine(
                "Limelight + Pedro ready"
        );

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

        LLResult result =
                limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            telemetry.addLine(
                    "No valid Limelight result"
            );
            return;
        }

        List<ColorResult> blobs =
                result.getColorResults();

        if (blobs.isEmpty()) {
            telemetry.addLine(
                    "No pollen detected"
            );
            return;
        }

        /*
         * Current behavior:
         * choose the largest visible pollen clump.
         *
         * If you want nearest individual ball instead,
         * change this line to:
         *
         * ColorResult best = findLargestBlob(blobs);
         */
        ColorResult best =
                findLargestClump(blobs);

        Pose current =
                follower.pose();

        /*
         * Limelight TX:
         * positive = target is to the right.
         *
         * We convert TX into radians and use the robot's
         * current Pedro heading to calculate the field heading.
         *
         * This assumes the Limelight camera is facing forward
         * with approximately zero yaw offset.
         */
        double tx =
                Math.toRadians(
                        best.getTargetXDegrees()
                );

        double targetHeading =
                current.heading() - tx;


        /*
         * For the first test, only drive 12 inches
         * toward the target.
         *
         * Then we scan again.
         */
        double driveDistance = 12.0;


        double targetX =
                current.x()
                        + driveDistance
                        * Math.cos(targetHeading);

        double targetY =
                current.y()
                        + driveDistance
                        * Math.sin(targetHeading);


        Pose target =
                p.of(
                        targetX,
                        targetY,
                        Math.toDegrees(targetHeading)
                );


        Path visionPath =
                line(current, target)
                        .constant(target);


        follower.follow(visionPath);

        driving = true;

        // This counts actual vision-driven movements,
        // not loop iterations.
        visionCycles++;
    }


    private ColorResult findLargestClump(
            List<ColorResult> blobs
    ) {

        ColorResult bestBlob = null;
        double bestScore = 0;


        for (ColorResult center : blobs) {

            double score = 0;


            for (ColorResult other : blobs) {

                double dx =
                        Math.abs(
                                center.getTargetXDegrees()
                                        - other.getTargetXDegrees()
                        );

                double dy =
                        Math.abs(
                                center.getTargetYDegrees()
                                        - other.getTargetYDegrees()
                        );


                /*
                 * Treat blobs that are close together in
                 * the camera view as belonging to one clump.
                 */
                if (dx < 8 && dy < 8) {
                    score +=
                            other.getTargetArea();
                }
            }


            if (score > bestScore) {
                bestScore = score;
                bestBlob = center;
            }
        }


        return bestBlob;
    }


    private ColorResult findLargestBlob(
            List<ColorResult> blobs
    ) {

        ColorResult best =
                blobs.get(0);


        for (ColorResult blob : blobs) {

            if (blob.getTargetArea()
                    > best.getTargetArea()) {

                best = blob;
            }
        }


        return best;
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
        if (!driving &&
                visionCycles < MAX_VISION_CYCLES) {

            scanAndDrive();
        }


        /*
         * Telemetry
         */
        LLResult result =
                limelight.getLatestResult();


        if (result != null && result.isValid()) {

            List<ColorResult> blobs =
                    result.getColorResults();


            telemetry.addData(
                    "Pollen blobs",
                    blobs.size()
            );


            if (!blobs.isEmpty()) {

                ColorResult best =
                        findLargestClump(blobs);


                telemetry.addData(
                        "Best TX",
                        "%.2f",
                        best.getTargetXDegrees()
                );


                telemetry.addData(
                        "Best TY",
                        "%.2f",
                        best.getTargetYDegrees()
                );


                telemetry.addData(
                        "Best Area",
                        "%.2f%%",
                        best.getTargetArea()
                );
            }

        } else {

            telemetry.addData(
                    "Limelight",
                    "No valid result"
            );
        }


        telemetry.addData(
                "Vision cycles",
                visionCycles + " / " + MAX_VISION_CYCLES
        );


        telemetry.addData(
                "Driving",
                driving
        );


        telemetry.addData(
                "Robot X",
                "%.2f",
                follower.pose().x()
        );


        telemetry.addData(
                "Robot Y",
                "%.2f",
                follower.pose().y()
        );


        telemetry.addData(
                "Heading",
                "%.1f",
                Math.toDegrees(
                        follower.pose().heading()
                )
        );


        telemetry.addData(
                "Pedro Busy",
                follower.isBusy()
        );


        telemetry.update();
    }


    @Override
    public void stop() {

        if (limelight != null) {
            limelight.stop();
        }
    }
}