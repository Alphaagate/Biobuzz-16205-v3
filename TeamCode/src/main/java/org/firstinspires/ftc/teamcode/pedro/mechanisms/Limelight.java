package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@TeleOp(name = "Austin Limelight Ball Test")
public class Limelight extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        /*
         * Use the class field.
         *
         * Your old code did:
         *
         * Limelight3A limelight =
         *     hardwareMap.get(...);
         *
         * which created a new local variable and
         * shadowed the class field.
         */
        limelight =
                hardwareMap.get(
                        Limelight3A.class,
                        "limelight"
                );


        // Pollen pipeline
        limelight.pipelineSwitch(0);

        limelight.start();

        waitForStart();


        while (opModeIsActive()) {

            LLResult result =
                    limelight.getLatestResult();


            if (result != null &&
                    result.isValid()) {


                List<LLResultTypes.ColorResult> blobs =
                        result.getColorResults();


                telemetry.addData(
                        "Balls detected",
                        blobs.size()
                );


                for (int i = 0;
                     i < blobs.size();
                     i++) {


                    LLResultTypes.ColorResult blob =
                            blobs.get(i);


                    telemetry.addData(
                            "Ball " + i + " TX",
                            "%.2f°",
                            blob.getTargetXDegrees()
                    );


                    telemetry.addData(
                            "Ball " + i + " TY",
                            "%.2f°",
                            blob.getTargetYDegrees()
                    );


                    telemetry.addData(
                            "Ball " + i + " Area",
                            "%.2f%%",
                            blob.getTargetArea()
                    );
                }


            } else {


                telemetry.addData(
                        "Limelight",
                        "No valid result"
                );
            }


            telemetry.update();
        }


        limelight.stop();
    }
}