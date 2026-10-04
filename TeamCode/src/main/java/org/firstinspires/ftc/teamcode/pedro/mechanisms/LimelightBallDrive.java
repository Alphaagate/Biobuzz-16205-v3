package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@TeleOp(name = "Limelight Ball Drive")
public class LimelightBallDrive extends OpMode {

    private Follower follower;
    private Limelight3A limelight;

    // -----------------------------
    // TUNING VALUES
    // -----------------------------

    // How strongly the robot turns toward the ball.
    private static double TURN_KP = 0.035;

    // Maximum turning power.
    private static double MAX_TURN = 0.45;

    // Maximum forward power.
    private static double MAX_FORWARD = 0.35;

    // Ignore tiny horizontal errors.
    private static double TX_DEADBAND = 1.5;

    // The ball's area when we want to stop.
    // YOU WILL NEED TO TUNE THIS.
    private static double STOP_AREA = 1.0;

    // How strongly target area affects forward speed.
    private static double AREA_KP = 0.40;


    @Override
    public void init() {

        // Create Pedro follower.
        follower = Constants.create(hardwareMap);

        // Get Limelight.
        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.setPollRateHz(100);

        // IMPORTANT:
        // Your current Limelight.java uses pipeline 5
        // for your ball/pollen detection.
        limelight.pipelineSwitch(5);

        // Start Limelight polling.
        limelight.start();

        telemetry.addLine("Ready");
        telemetry.addLine("Hold A to chase the ball");
        telemetry.update();
    }


    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        // ----------------------------------
        // DEFAULT: ROBOT STAYS STOPPED
        // ----------------------------------

        double forward = 0;
        double lateral = 0;
        double turn = 0;


        // ----------------------------------
        // ONLY CHASE WHILE A IS HELD
        // ----------------------------------

        if (gamepad1.a) {

            LLResultTypes.ColorResult target = null;

            if (result != null && result.isValid()) {

                List<LLResultTypes.ColorResult> blobs =
                        result.getColorResults();

                // ----------------------------------
                // FIND THE LARGEST BALL
                // ----------------------------------

                for (LLResultTypes.ColorResult blob : blobs) {

                    if (target == null ||
                            blob.getTargetArea() > target.getTargetArea()) {

                        target = blob;
                    }
                }


                // ----------------------------------
                // BALL FOUND
                // ----------------------------------

                if (target != null) {

                    double tx = target.getTargetXDegrees();
                    double area = target.getTargetArea();


                    // ----------------------------------
                    // TURN TOWARD BALL
                    // ----------------------------------

                    if (Math.abs(tx) > TX_DEADBAND) {

                        /*
                         * Limelight:
                         *     positive TX = ball is to the right
                         *
                         * Pedro:
                         *     positive turn = counterclockwise
                         *
                         * Therefore we use -tx.
                         */

                        turn = -tx * TURN_KP;

                        turn = Range.clip(
                                turn,
                                -MAX_TURN,
                                MAX_TURN
                        );
                    }


                    // ----------------------------------
                    // DRIVE TOWARD BALL
                    // ----------------------------------

                    /*
                     * Far ball:
                     *     small area -> drive faster
                     *
                     * Close ball:
                     *     large area -> slow down
                     *
                     * Stop when area reaches STOP_AREA.
                     */

                    if (area < STOP_AREA) {

                        forward =
                                (STOP_AREA - area) * AREA_KP;

                        forward = Range.clip(
                                forward,
                                0,
                                MAX_FORWARD
                        );
                    }


                    telemetry.addData(
                            "BALL",
                            "FOUND"
                    );

                    telemetry.addData(
                            "TX",
                            "%.2f degrees",
                            tx
                    );

                    telemetry.addData(
                            "Area",
                            "%.3f%%",
                            area
                    );

                    telemetry.addData(
                            "Forward",
                            "%.2f",
                            forward
                    );

                    telemetry.addData(
                            "Turn",
                            "%.2f",
                            turn
                    );

                } else {

                    telemetry.addData(
                            "BALL",
                            "No color targets"
                    );
                }

            } else {

                telemetry.addData(
                        "BALL",
                        "Limelight invalid"
                );
            }
        }


        // ----------------------------------
        // SEND COMMAND TO PEDRO
        // ----------------------------------

        follower.manual(
                forward,
                lateral,
                turn
        );

        follower.update();


        telemetry.addData(
                "Follower Mode",
                follower.mode()
        );

        telemetry.update();
    }


    @Override
    public void stop() {

        // Make absolutely sure the robot stops.
        follower.manual(
                0,
                0,
                0
        );

        follower.update();

        limelight.stop();
    }
}