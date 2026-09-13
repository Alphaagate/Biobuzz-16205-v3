package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.path;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;

import com.pedropathing.math.Pose;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.paths.Path;

import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class DriveTest extends OpMode {


    protected Command updateShooter;
    private int gateCycleNum = -1;
    private DcMotorEx outtake, outtake2;

    private final PoseFactory p = PoseFactory.degrees();
    //    private final PoseFactory p = PoseFactory.degrees().mirrorX(70.75);
//TODO: red and blue side switch, test which one is for red side and determine if you want to use control pad or make a separate opmode
    private final Pose startPose = p.of(22.55, 116.45, 180);

    protected Pose shootingPose = p.of(53.423, 74.143, 0);

    protected Pose middlePickupPose = p.of(13.990, 55, 180);
    protected Pose middlePickupControlPoint2 = p.of(42, 57, 0);
    protected Pose closePickupPose = p.of(20.590, 81.860, 180);

    private double xOffset = -0.4;
    private double yOffset = 1;

    protected Pose[] gatePickupPoses = {
            p.of(13.3 + xOffset, 56 + yOffset, 150),
            p.of(13.3 + xOffset, 56.25 + yOffset, 150),
            p.of(13.3 + xOffset, 56.5 + yOffset, 150),
            p.of(13.3 + xOffset, 56.75 + yOffset, 150),
            p.of(13.3 + xOffset, 57 + yOffset, 150),
    };

    protected Pose farPickupPose = p.of(11.590, 33.210, 180);
    protected Pose farPickupControlPoint = p.of(45, 34, 0);

    protected Pose cornerPose = p.of(13.990, 17.860, 210);
    protected Pose cornerBackupPose = p.of(11.690, 8.360, 180);

    protected Pose farShootingPose = p.of(51.247, 10.099, 0);
    protected Pose parkPose = p.of(45.747, 15.099, 0);

    protected Pose closeParkPose = p.of(56.990, 102.860, 180);
    private Follower follower;
    private Limelight3A limelight;

    public double vel;

    protected Path shootPreloads;
    protected Path pickupMiddle;
    protected Path shootMiddle;
    protected Path[] pickupGates;
    protected Path[] shootGates;
    protected Path shootGateAndPark;
    protected Path pickupClose;
    protected Path shootClose;
    protected Path shootCloseAndPark;
    protected Path pickupFar;
    protected Path shootFar;
    protected Path shootFarAndPark;
    protected Path pickupCorner;
    protected Path backupCorner;
    protected Path shootCorner;
    protected Path park;
    protected Path shootCornerClose;

//    private Path park() {
//        return line(startPose, park).linear(startPose, park);
//    }

    //TODO: see if this works
    protected void createAutoCommands() {
        double shootTime = 150;

        schedule(
                sequential(
//                        shootPreloads(),
                        race(
//                                waitUntil(() -> robot.isShooterReady()),
                                waitMs(500)
                        ),
                        runCycle(pickupMiddle, shootMiddle, shootTime, 700, 600),
                        gateCycle(shootTime, 1000),
                        gateCycle(shootTime, 1500),
                        runCycle(pickupClose, shootClose, shootTime, 900, 500),
                        gateCycle(shootTime, 1000),
                        gateCycle(shootTime, 1500),
                        runCycle(pickupFar, shootFarAndPark, shootTime + 125, 700, 750),
//                        shootAndSetIntaking(),
                        waitMs(500)
//                        robot.setIntakePower(0),
//                        robot.deactivateShooter()
                )
        );
    }


//    protected Command shootPreloads() {
//        return sequential(startFlywheel(), follow(robot.getFollower(), shootPreloads));
//    }

    protected Command runCycle(Path pickupPath, Path shootPath, double shootDelayMs,
                               double intakeDelayMs, double shootingDelayMs) {
        return sequential(
                parallel(
                        sequential(
                                waitMs(shootDelayMs),
                                parallel(
                                        follow(follower, pickupPath),
                                        sequential(
                                                waitMs(intakeDelayMs)
//                                                shootFar.setIntakePower(1)
                                        )
                                )//.raceWith(waitUntil(() -> shootFar.beamBroken()))

                                // TODO: remember to add actual functionality for parallel

                        )
                ),
                parallel(
                        sequential(
                                waitMs(200)
//                                conditional(
//                                        //() -> shootFar.beamBroken(),
//                                        instant(() -> {}), // do nothing
//                                        sequential(
//                                                shootFar.setIntakePower(-1),
//                                                waitMs(50),
//                                                shootFar.setIntakePower(0)
//                                        )
//                                )
                        ),
                        follow(follower, shootPath),
                        sequential(
                                waitMs(shootingDelayMs)
//                                shootFar.setIntakePower(0)
                        )
                )
        );
    }

    protected Command gateCycle(double shootDelayMs, double gateWaitMs) {
        gateCycleNum++;

        return sequential(
                parallel(
                        sequential(
                                waitMs(shootDelayMs),
                                follow(follower, pickupGates[gateCycleNum])
                        )
                ),
                race(
                        waitMs(gateWaitMs)
//                        waitUntil(() -> robot.beamBroken())
                ),
                parallel(
                        sequential(
                                waitMs(200)
//                                conditional(
//                                        () -> robot.beamBroken(),
//                                        instant(() -> {}),
//                                        sequential(
//                                                waitMs(50)
//                                        )
//                                )
                        ),
                        follow(follower, shootGates[gateCycleNum]),
                        sequential(
                                waitMs(200)
                        )
                )
        );
    }


//    protected Command gateCycleAndPark(double shootDelayMs, double gateWaitMs) {
//        gateCycleNum++;
//        return sequential(
//                        sequential(waitMs(shootDelayMs), robot.setIntakePower(1),
//                                follow(follower, pickupGates[gateCycleNum]))
//                waitMs(gateWaitMs), parallel(follow(follower, shootGateAndPark)
//                        sequential(waitMs(1000), robot.setIntakePower(0))));
//    }


//    public static Command turnTo(Follower follower, double radians) {
//        return new CommandBuilder().setStart(() -> {
//            Pose pose = follower.pose();
//            Path path = new Path(new BezierPoint(pose));
//            path.setHeadingInterpolation(HeadingInterpolator.constant(radians));
//            follower.followPath(path);
//        }).setDone(() -> !follower.isBusy());
//    }

//    protected Command shootAndSetIntaking() {
//        return instant(() -> robot.setState(States.SHOOTING));
//    }
//
//    protected Command startFlywheel() {
//        return instant(() -> robot.activateShooter());
//    }


    private void generatePaths() {

        shootPreloads = line(startPose, shootingPose)
                .reverseTangent();

        pickupMiddle = curve(
                shootingPose,
                middlePickupControlPoint2,
                middlePickupPose
        ).tangent();

        shootMiddle = line(middlePickupPose, shootingPose)
                .reverseTangent();

        pickupGates = new Path[gatePickupPoses.length];
        shootGates = new Path[gatePickupPoses.length];

        for (int i = 0; i < gatePickupPoses.length; i++) {

            pickupGates[i] = line(shootingPose, gatePickupPoses[i])
                    .heading(
                            Interpolator.piecewise()
                                    .until(0.6, Interpolator.tangent)
                                    .until(
                                            1,
                                            Interpolator.constant(gatePickupPoses[i])
                                    )
                    );

            shootGates[i] = line(gatePickupPoses[i], shootingPose)
                    .reverseTangent();
        }

        shootGateAndPark = line(
                gatePickupPoses[gatePickupPoses.length - 1],
                closeParkPose
        ).reverseTangent();

        pickupClose = line(shootingPose, closePickupPose)
                .constant(shootingPose);

        shootClose = line(closePickupPose, shootingPose)
                .constant(shootingPose);

        shootCloseAndPark = line(closePickupPose, closeParkPose)
                .constant(shootingPose);

        pickupFar = curve(
                shootingPose,
                farPickupControlPoint,
                farPickupPose
        ).tangent();

        shootFar = line(farPickupPose, shootingPose)
                .constant(shootingPose);

        shootFarAndPark = line(farPickupPose, closeParkPose)
                .reverseTangent();

        pickupCorner = line(shootingPose, cornerPose)
                .constant(cornerPose);

        backupCorner = line(cornerPose, cornerBackupPose)
                .linear(cornerPose, cornerBackupPose);

        shootCorner = line(cornerBackupPose, farShootingPose)
                .constant(cornerBackupPose);

        park = line(farShootingPose, parkPose)
                .tangent();

        shootCornerClose = line(cornerBackupPose, closeParkPose)
                .constant(cornerBackupPose);
    }


    @Override
    public void init() {

        outtake = hardwareMap.get(DcMotorEx.class, "o1");
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);

        outtake2 = hardwareMap.get(DcMotorEx.class, "o2");
        outtake2.setDirection(DcMotorSimple.Direction.FORWARD);

        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.setPollRateHz(100);

        // Pollen pipeline
        limelight.pipelineSwitch(0);

        limelight.start();

        generatePaths();

        telemetry.addLine("Initialized - Ready!");
        telemetry.update();
    }


    public void start() {
//        schedule(follow(follower, park()));
//        may need but i think is already replaced by create auto commands

        createAutoCommands();
    }


    @Override
    public void loop() {

        follower.update();

        Scheduler.execute();

        double velocity = outtake.getVelocity();
        double error = vel - velocity;

        double feedback = error * 0.005;
        double feedforward = 0.00036 * vel + 0.08;

        outtake.setPower(feedback + feedforward);
        outtake2.setPower(feedback + feedforward);

        LLResult result = limelight.getLatestResult();

        telemetry.addData(
                "Follower Busy",
                follower.isBusy()
        );

        telemetry.addData(
                "X",
                follower.pose().x()
        );

        telemetry.addData(
                "Y",
                follower.pose().y()
        );

        telemetry.addData(
                "Heading (deg)",
                Math.toDegrees(follower.pose().heading())
        );

        // Only added to let you verify that the Limelight is actually returning data.
        if (result != null && result.isValid()) {
            telemetry.addData(
                    "Limelight",
                    "Valid"
            );

            telemetry.addData(
                    "Pollen blobs",
                    result.getColorResults().size()
            );
        } else {
            telemetry.addData(
                    "Limelight",
                    "No valid result"
            );
        }

        telemetry.update();
    }
}