package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.ColorResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/*
 * Drop-in replacement for LimelightPollenAuto that does NOT use Pedro Pathing
 * (no Follower, no Path, no Pose, no localizer).
 *
 * Instead of planning a field-frame path to a computed target point, this
 * drives the mecanum base directly, every loop, using simple proportional
 * control on the Limelight's tx (bearing) and target area (a proxy for
 * distance, since ColorResult does not expose a real range/depth value).
 *
 * Motor names/directions below are taken from your Constants.java
 * (MecanumConfig: fL, fR, bL, bR; front-left + back-left REVERSE,
 * front-right + back-right FORWARD).
 *
 * IMPORTANT: TURN_KP, DRIVE_KP, TARGET_AREA, and the clip limits below are
 * placeholders. There is no way to know the right values without testing on
 * your actual robot/camera mount -- start small and increase gains until the
 * approach is responsive but not oscillating.
 */
@Autonomous(name = "BIOBUZZ Limelight Pollen Test (No Pedro)")
public class LimelightPollenAuto_NoPedro extends LinearOpMode {

    // Drive motors (names/directions from Constants.MecanumConfig)
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private Limelight3A limelight;

    // ---- Tunable constants (PLACEHOLDERS -- you must test and adjust) ----
    private static final double TURN_KP = 0.02;      // power per degree of tx error
    private static final double DRIVE_KP = 0.01;     // power per unit of area error
    private static final double TARGET_AREA = 5.0;   // % area considered "close enough" -- guess, tune on your robot
    private static final double MAX_TURN_POWER = 0.35;
    private static final double MAX_DRIVE_POWER = 0.35;
    private static final double AREA_STOP_TOLERANCE = 0.5; // stop driving forward within this band of TARGET_AREA

    @Override
    public void runOpMode() {

//        // ---- Hardware init (mirrors Constants.drivetrainConfig) ----
//        frontLeft  = hardwareMap.get(DcMotor.class, "fL");
//        frontRight = hardwareMap.get(DcMotor.class, "fR");
//        backLeft   = hardwareMap.get(DcMotor.class, "bL");
//        backRight  = hardwareMap.get(DcMotor.class, "bR");

//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---- Limelight init ----
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        boolean pipelineChanged = limelight.pipelineSwitch(5);

        limelight.start();

        telemetry.addData("Pipeline switch", pipelineChanged);
        telemetry.addData("Connected", limelight.isConnected());
        telemetry.addData("Running", limelight.isRunning());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Connected", limelight.isConnected());
            telemetry.addData("Running", limelight.isRunning());
            telemetry.addData("Time Since Update", "%d ms",
                    limelight.getTimeSinceLastUpdate());
            telemetry.addData("Connection Info", limelight.getConnectionInfo());

            LLResult result = limelight.getLatestResult();

            if (result != null) {

                telemetry.addData("Result Valid", result.isValid());

                if (result.isValid()) {

                    List<ColorResult> blobs = result.getColorResults();

                    telemetry.addData("Blobs Seen", blobs.size());

                    if (!blobs.isEmpty()) {

                        ColorResult best = blobs.get(0);

                        double tx = best.getTargetXDegrees();
                        double area = best.getTargetArea();

                        telemetry.addData("TX", "%.2f", tx);
                        telemetry.addData("TY", "%.2f", best.getTargetYDegrees());
                        telemetry.addData("Area", "%.2f%%", area);

                    } else {
                        telemetry.addLine("Connected, but no pollen detected");
                    }

                } else {
                    telemetry.addLine("Limelight connected, but result is invalid");
                }

            } else {
                telemetry.addLine("getLatestResult() returned null");
            }

            telemetry.update();
        }
//        mecanumDrive(0, 0, 0);
        limelight.stop();
    }

    /**
     * Direct mecanum drive, no localizer/path planning involved.
     * axial: forward(+)/back(-), lateral: strafe right(+)/left(-), yaw: turn CW(+)/CCW(-)
     */
    private void mecanumDrive(double axial, double lateral, double yaw) {
        double flPower = axial + lateral + yaw;
        double frPower = axial - lateral - yaw;
        double blPower = axial - lateral + yaw;
        double brPower = axial + lateral - yaw;

        double max = Math.max(1.0, Math.max(Math.abs(flPower),
                Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower)))));

        frontLeft.setPower(flPower / max);
        frontRight.setPower(frPower / max);
        backLeft.setPower(blPower / max);
        backRight.setPower(brPower / max);
    }
}