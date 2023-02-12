package org.WaialuaRobotics359.robot.commands.autonomous;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.LimeLight;
import org.WaialuaRobotics359.robot.subsystems.Swerve;
import org.WaialuaRobotics359.robot.commands.swerve.TeleopSwerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class AutoLimelightAlign extends CommandBase {

    private LimeLight s_limelight;
    private Swerve s_swerve;

    ProfiledPIDController txPid = new ProfiledPIDController(0, 0, 0, null);
    ProfiledPIDController tyPid = new ProfiledPIDController(0, 0, 0, null);

    double tx;
    double ty;

    public AutoLimelightAlign(LimeLight s_limelight, Swerve s_swerve) {
        this.s_limelight = s_limelight;
        this.s_swerve = s_swerve;
    }

    private void fetchValues() {
       tx = s_limelight.getTX();
       ty = s_limelight.getTY();
    }

    @Override
    public void initialize() {
        fetchValues();
        txPid.reset(tx);
        tyPid.reset(ty);
    }

    @Override
    public void execute() {
       fetchValues();

       CommandScheduler.getInstance().schedule(
           new TeleopSwerve(
               s_swerve, null, null, null, null
           )
       );
    }
    
    @Override
    public boolean isFinished(){
        return true;
    }

    @Override 
    public void end(boolean interupted) {
        // yolo
    }
}