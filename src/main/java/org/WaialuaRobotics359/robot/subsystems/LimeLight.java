package org.WaialuaRobotics359.robot.subsystems;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Constants.Limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {
    private NetworkTable NetworkTable;

    public LimeLight() {
        NetworkTable = edu.wpi.first.networktables.NetworkTableInstance.getDefault().getTable("limelight");
        ConfigStart();
    }

    public void ConfigStart(){
        setPipeline(Constants.Limelight.Options.RetroReflective);
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

    public void setLEDs(int LEDmode){
        NetworkTable.getEntry("ledMode").setNumber(LEDmode);
    }

    public void setCAM(int CAMmode){
        NetworkTable.getEntry("camMode").setNumber(CAMmode);
    }

    public Double getDistance() {
        return (Limelight.limelightHeight - Limelight.nodeHeight) / Math.tan(Math.toRadians(getTY()));
    }

    @Override
    public void periodic() {
    }    
}
