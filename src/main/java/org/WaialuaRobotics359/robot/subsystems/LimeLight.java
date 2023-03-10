package org.WaialuaRobotics359.robot.subsystems;

import org.WaialuaRobotics359.lib.math.Conversions;
import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Constants.Limelight;
import org.WaialuaRobotics359.robot.util.LimelightHelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {
    private NetworkTable NetworkTable;

    private boolean DriverMode = false;
    private int previousPipeline;

    public LimeLight() {
        NetworkTable = edu.wpi.first.networktables.NetworkTableInstance.getDefault().getTable("limelight");
        ConfigStart();
        System.out.println("LimelightConfig");
    }

    public void ConfigStart(){
        setPipeline(2);
        setLEDs(Constants.Limelight.Options.LEDPipe);
        setCAM(Constants.Limelight.Options.CAMVision);


    }

    public double getTX() {
        return NetworkTable.getEntry("tx").getDouble(0);
    }

    public double getTY() {
        return NetworkTable.getEntry("ty").getDouble(0);
    }

    public boolean hasTarget() {
        return (NetworkTable.getEntry("tv").getDouble(0) >.5);
    }

    public void setPipeline(int pipeline) {
        NetworkTable.getEntry("pipeline").setNumber(pipeline);
    }

    public double getPipeline(){
        return NetworkTable.getEntry("pipeline").getDouble(0);
    }

    public Pose2d getPose2d(){
        if (Conversions.isBetween((LimelightHelpers.getFiducialID("limelight")), 0, 12) && LimelightHelpers.getTV("limelight") == 1){
            return DriverStation.getAlliance() == DriverStation.Alliance.Red ? LimelightHelpers.getBotPose2d_wpiRed("limelight") : LimelightHelpers.getBotPose2d_wpiBlue("limelight");
        }else{
            return null;
        }
    }


    public void setLEDs(int LEDmode){
        NetworkTable.getEntry("ledMode").setNumber(LEDmode);
    }

    public void setCAM(int CAMmode){
        NetworkTable.getEntry("camMode").setNumber(CAMmode);
    }

    public Double getDistance() {
        return (Limelight.limelightHeight - Limelight.nodeHeight) / Math.tan(Math.toRadians(getTY()));
    }

    public void toggleDriver(){
        if((int)getPipeline() != 4 && DriverMode){
            DriverMode = false; 
        }
        
        if(!DriverMode){previousPipeline = (int)getPipeline();}

        if(!DriverMode){
            setPipeline(4);
            DriverMode = true; 
        } else {
            setPipeline(previousPipeline);
            DriverMode = false; 
        }
    }
    
    public boolean GetDriverMode(){
        return DriverMode;
    }

    @Override
    public void periodic() {
    }    
}
