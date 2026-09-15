package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * AprilTagRecognition wraps a VisionPortal + AprilTagProcessor pointed at a USB webcam
 * (e.g. a Logitech C920/C270). It is a plain helper class -- NOT an OpMode -- so you can
 * create one instance in init() and read from it every loop() of any OpMode.
 *
 * The pose values you care about (range, bearing, yaw, elevation) are computed by the FTC
 * SDK itself from the camera's calibration + the tag's known physical size in the built-in
 * tag library -- you never calculate them by hand. You just read them off
 * detection.ftcPose after calling getDetections() or getTag(id).
 *
 * Usage:
 *   AprilTagRecognition aprilTag = new AprilTagRecognition(hardwareMap, "Webcam 1");
 *   aprilTag.setManualExposure(6, 250);   // optional: reduces motion blur while driving
 *   ...
 *   AprilTagDetection tag = aprilTag.getTag(DESIRED_TAG_ID);
 *   if (tag != null) {
 *       double range   = tag.ftcPose.range;    // inches
 *       double bearing = tag.ftcPose.bearing;  // degrees, +ve = tag is to the left
 *       double yaw     = tag.ftcPose.yaw;      // degrees, tag's own rotation
 *   }
 *   ...
 *   aprilTag.close();   // call when the OpMode ends, e.g. in stop()
 */
public class AprilTagRecognition {

    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;

    /**
     * @param hardwareMap the OpMode's hardwareMap
     * @param webcamName  the exact name configured in the robot config (e.g. "Webcam 1")
     */
    public AprilTagRecognition(HardwareMap hardwareMap, String webcamName) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    /** All currently visible AprilTag detections (may be empty, never null). */
    public List<AprilTagDetection> getDetections() {
        return aprilTagProcessor.getDetections();
    }

    /**
     * Finds one specific tag ID among the current detections.
     * @return the detection, or null if that tag is not currently visible / not in the tag library
     */
    public AprilTagDetection getTag(int id) {
        for (AprilTagDetection detection : getDetections()) {
            if (detection.metadata != null && detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    public boolean isTagVisible(int id) {
        return getTag(id) != null;
    }

    /**
     * Locks the webcam to a short, fixed exposure and higher gain. This trades a slightly
     * noisier/darker image for much less motion blur, which makes tag detection far more
     * reliable while the robot is moving. Call this once, after construction.
     *
     * Safe defaults to start from: exposureMS = 6, gain = 250 (matches the FTC sample
     * ConceptAprilTagOptimizeExposure). Tune per-camera/lighting if needed.
     */
    public void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            return;
        }

        // Wait (with a timeout) for the camera to actually start streaming before
        // touching its controls -- setting them too early silently does nothing.
        long deadline = System.currentTimeMillis() + 2500;
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING
                && System.currentTimeMillis() < deadline) {
            sleepQuiet(20);
        }
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return; // camera never came up; leave exposure alone rather than throw
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleepQuiet(50);
        }
        exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
        sleepQuiet(20);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        sleepQuiet(20);
    }

    private void sleepQuiet(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /** Call when your OpMode is stopping (e.g. from stop()) to free the camera. */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }
}