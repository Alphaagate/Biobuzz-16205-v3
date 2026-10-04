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

import com.pedropathing.math.Pose;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.interpolator.Interpolator;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class DriveTest extends OpMode {


    protected Command updateShooter;
    private int gateCycleNum = -1;
    private DcMotorEx outtake, outtake2;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    //    private final PoseFactory p = PoseFactory.degrees().mirrorX(70.75);
//TODO: red and blue side switch, test which one is for red side and determine if you want to use control pad or make a separate opmode
    private final Pose startPose = poseFactory.of(48, 8, 90);
    private final Pose pickupCornerStartPose = poseFactory.of(48, 8, 90);
    private final Pose pickupCornerPose = poseFactory.of(8, 8, 180);
    private final Pose shootFarPose = poseFactory.of(47.6008, 129.3427, 90);
    private final Pose shootFarControl1Pose = poseFactory.of(37.3307, 103.2638, 0);
    private final Pose intakeFlowerPose = poseFactory.of(12, 48, 180);
    private final Pose shootFlowerPose = poseFactory.of(48, 8, 90);
    private final Pose pickupLimelightPose = poseFactory.of(59.8554, 51.0381, 74.5991);
    private final Pose shootLimelightPose = poseFactory.of(48.3992, 8.9979, 90);
    private final Pose parkPose = poseFactory.of(9.0867, 101.5183, 90);




    private Follower follower;
    private Limelight3A limelight;

    public double vel;
    protected Path start;
    protected Path shootPreloads;
    protected Path pickupCorner;
    protected Path shootFar;
    protected Path intakeFlower;
    protected Path shootFlower;
    protected Path pickupLimelight;
    protected Path shootLimelight;
    protected Path park;


//    private Path park() {
//        return line(start, park).linear(start, park);
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
                        runCycle(start, shootPreloads, shootTime, 700, 600),
                        runCycle(pickupCorner, shootFar, shootTime, 900, 500),
                        runCycle(intakeFlower, shootFlower, shootTime + 125, 700, 750),
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

        shootPreloads = line(startPose, pickupCornerStartPose)
                .linear(startPose, pickupCornerStartPose);
        pickupCorner = line(pickupCornerStartPose, pickupCornerPose)
                .linear(pickupCornerStartPose, pickupCornerPose);
        shootFar =  curve(pickupCornerPose, shootFarControl1Pose, shootFarPose)
                .linear(pickupCornerPose, shootFarPose);
        intakeFlower = line(shootFarPose, intakeFlowerPose)
                .linear(shootFarPose, intakeFlowerPose);
        shootFlower = line(shootFarPose, intakeFlowerPose)
                .linear(shootFarPose, intakeFlowerPose);
        pickupLimelight = line(shootFlowerPose, pickupLimelightPose)
                .tangent();
        shootLimelight = line(pickupLimelightPose, shootLimelightPose)
                .constant(shootLimelightPose);
        park = line(shootLimelightPose, parkPose)
                .linear(shootLimelightPose, parkPose);

//        pickupGates = new Path[gatePickupPoses.length];
//        shootGates = new Path[gatePickupPoses.length];

//        for (int i = 0; i < gatePickupPoses.length; i++) {
//
//            pickupGates[i] = line(shootingPose, gatePickupPoses[i])
//                    .heading(
//                            Interpolator.piecewise()
//                                    .until(0.6, Interpolator.tangent)
//                                    .until(
//                                            1,
//                                            Interpolator.constant(gatePickupPoses[i])
//                                    )
//                    );
//
//            shootGates[i] = line(gatePickupPoses[i], shootingPose)
//                    .reverseTangent();
//        }
//
//        shootGateAndPark = line(
//                gatePickupPoses[gatePickupPoses.length - 1],
//                closeParkPose
//        ).reverseTangent();


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