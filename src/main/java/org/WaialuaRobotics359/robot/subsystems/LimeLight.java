package org.WaialuaRobotics359.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {
    private NetworkTable NetworkTable;

    private LimeLight() {
        NetworkTable = edu.wpi.first.networktables.NetworkTableInstance.getDefault().getTable("limelight");
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

    public double getTargetLatency() {
        return NetworkTable.getEntry("tl").getDouble(0);
    }

    public void setPipeline(int pipeline) {
        NetworkTable.getEntry("pipeline").setNumber(pipeline);
    }

    @Override
    public void periodic() {

            NetworkTable.getEntry("ledMode").setNumber(mPeriodicIO.ledMode);
            NetworkTable.getEntry("camMode").setNumber(mPeriodicIO.camMode);
            NetworkTable.getEntry("pipeline").setNumber(mPeriodicIO.pipeline);
            NetworkTable.getEntry("stream").setNumber(mPeriodicIO.stream);
            NetworkTable.getEntry("snapshot").setNumber(mPeriodicIO.snapshot);

    }


    
}
