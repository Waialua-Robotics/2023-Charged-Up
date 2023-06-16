package org.WaialuaRobotics359.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Constants.Limelight;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
    private PhotonCamera camera;

    private Transform3d robotToCam;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator photonPoseEstimator;

    public PhotonVision() {
        camera = new PhotonCamera("camera1");
        robotToCam = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0,0,0)); //insert camera pos. fromm center
        try { 
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } 
        catch (IOException error) {
            error.printStackTrace();
        }
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);

        ConfigStart();
        System.out.println("PhotonConfig");
    }

    public void ConfigStart(){
        setPipeline(2);
        setLEDs(Constants.Limelight.Options.LEDPipe);
    }

    public double getYaw() {
        if (!camera.getLatestResult().hasTargets()) {
            return 0;
        }
        return camera.getLatestResult().getBestTarget().getYaw();
    }

    public double getPitch() {
        if (!camera.getLatestResult().hasTargets()) {
            return 0;
        }
        return camera.getLatestResult().getBestTarget().getPitch();
    }

    public boolean hasTarget() {
        return camera.getLatestResult().hasTargets();
    }

    public void setPipeline(int pipeline) {
        camera.setPipelineIndex(pipeline);
    }

    public double getPipeline(){
        return camera.getPipelineIndex();
    }

    public Optional<EstimatedRobotPose> getEstimatedPose(Pose2d previousPose) {
        photonPoseEstimator.setReferencePose(previousPose);
        return photonPoseEstimator.update();
    }

    public void setLEDs(int LEDmode){
        VisionLEDMode[] LEDstate = {VisionLEDMode.kDefault, VisionLEDMode.kOff, VisionLEDMode.kBlink, VisionLEDMode.kOn};
        camera.setLED(LEDstate[LEDmode]);
    }

    public double getDistance() {
        if (!camera.getLatestResult().hasTargets()) {
            return 0;
        }
        return (Limelight.limelightHeight - Limelight.nodeHeight) / Math.tan(Math.toRadians(getPitch()));
    }

    public void toggleDriver(){
        camera.setDriverMode(!camera.getDriverMode());
    }

    public boolean GetDriverMode(){
        return camera.getDriverMode();
    }

    @Override
    public void periodic() {
    }    
}
