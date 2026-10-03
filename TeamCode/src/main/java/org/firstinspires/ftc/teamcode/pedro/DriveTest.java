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
    private final Pose startPose = poseFactory.of(48, 8, 180);
//    private final Pose sPose = poseFactory.of(48, 8, 180);
    private final Pose cornerPose = poseFactory.of(8, 8, 180);
    private final Pose shootFarPose = poseFactory.of(47.6008, 129.3427, 90); //to shoot far side and pickup balls from flower
    private final Pose shootFarControlPose = poseFactory.of(37.3307, 103.2638, 0);
    private final Pose intakeFlowerPose = poseFactory.of(12, 48, 180);
    private final Pose shootClosePose = poseFactory.of(48, 8, 90);
    private final Pose limelightIntakePose = poseFactory.of(59.8554, 51.0381, 74.5991);
    private final Pose parkPose = poseFactory.of(9, 100, 90);





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

        shootPreloads = line(startPose, startPose)
                .reverseTangent();
        pickupCorner = line(shootClosePose, cornerPose)
                .constant(cornerPose);
        shootFar = line(shootFarPose, shootFarPose)
                .constant(shootingPose);
        intakeFlower = line(intakeFlowerPose, shootClosePose)
                .constant(flowerPose);
        shootFlower = line(shootClosePose, shootClosePose)
                .constant(shootingPose);
        pickupLimelight = line(limelightIntakePose, shootClosePose)
                .constant(limelightPose);
        shootLimelight = line(limelightPose, shootingPose)
                .constant(shootingPose);
        park = line(parkPose, parkPose)
                .tangent();

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

        shootCorner = line(cornerBackupPose, closeParkPose)
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
        follower.setPose(start);

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