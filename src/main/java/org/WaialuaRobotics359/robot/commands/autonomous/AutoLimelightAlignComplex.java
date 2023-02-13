
package org.WaialuaRobotics359.robot.commands.autonomous;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Constants.Limelight.txAlign;
import org.WaialuaRobotics359.robot.Constants.Limelight.tyAlign;
import org.WaialuaRobotics359.robot.subsystems.LimeLight;
import org.WaialuaRobotics359.robot.subsystems.Swerve;
import org.WaialuaRobotics359.robot.commands.swerve.TeleopSwerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class AutoLimelightAlignComplex extends CommandBase {

    private LimeLight s_limelight;
    private Swerve s_swerve;

    ProfiledPIDController txPid;
    ProfiledPIDController tyPid;
    //PIDController txPid;
    //PIDController tyPid;

    double tx;
    double ty;

    /* 
    DoubleSupplier translationSup = () -> tyPid.calculate(ty, 0);
    DoubleSupplier strafeSup = () -> txPid.calculate(tx, 0);
    DoubleSupplier rotationSup = () -> {return 0;};
    BooleanSupplier robotCentricSup = () -> {return false;};*/

    public AutoLimelightAlignComplex(LimeLight s_limelight, Swerve s_swerve) {
        this.s_limelight = s_limelight;
        this.s_swerve = s_swerve;
        txPid = new ProfiledPIDController(txAlign.kp, txAlign.ki, txAlign.kd, txAlign.profile);
        tyPid = new ProfiledPIDController(tyAlign.kp, tyAlign.ki, tyAlign.kd, tyAlign.profile);
        //txPid = new PIDController(txAlign.kp, txAlign.ki, txAlign.kd);
        //tyPid = new PIDController(tyAlign.kp, tyAlign.ki, tyAlign.kd);
        txPid.setTolerance(txAlign.threshold);
        tyPid.setTolerance(tyAlign.threshold);
    }

    private void fetchValues() {
       tx = -s_limelight.getTX();
       ty = s_limelight.getTY();
    }

    @Override
    public void initialize() {
        s_limelight.setPipeline(Constants.Limelight.Options.RetroReflective);
        fetchValues();
        txPid.reset(tx);
        tyPid.reset(ty);
        
        //txPid.reset();
        //tyPid.reset();
    }

    @Override
    public void execute() {
       fetchValues();
       
       //Translation2d translation = new Translation2d(txPid.calculate(-tx, 0), tyPid.calculate(-ty, 0));
       //Translation2d translation = new Translation2d(tyPid.calculate(-ty, 0), 0);
       Translation2d translation = new Translation2d(tyPid.calculate(-ty, -1), txPid.calculate(-tx, 0));
       SmartDashboard.putNumber("txPid", txPid.calculate(-tx, 0));
       SmartDashboard.putNumber("tyPid", tyPid.calculate(-ty, -1));

       /* 
        s_swerve.drive(
            translation, 0, false, true
        ); */

        /* 
       CommandScheduler.getInstance().schedule(
           new TeleopSwerve(
               s_swerve, 
               translationSup, 
               strafeSup, 
               rotationSup, 
               robotCentricSup
           )
       ); */
    }
    
    @Override
    public boolean isFinished(){        
        //SmartDashboard.putBoolean("at setpoint", (Math.abs(tx) < txAlign.threshold) && (Math.abs(ty) < tyAlign.threshold));
        return (Math.abs(tx) < txAlign.threshold) && (Math.abs(ty) < tyAlign.threshold);
    }

    @Override 
    public void end(boolean interupted) {
        s_swerve.stop();
    }
}