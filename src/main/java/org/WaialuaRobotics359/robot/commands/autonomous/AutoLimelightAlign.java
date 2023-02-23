package org.WaialuaRobotics359.robot.commands.autonomous;

import org.WaialuaRobotics359.lib.math.Conversions;
import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Constants.Limelight.txAlign;
import org.WaialuaRobotics359.robot.Constants.Limelight.tyAlign;
import org.WaialuaRobotics359.robot.subsystems.LimeLight;
import org.WaialuaRobotics359.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoLimelightAlign extends CommandBase {

    private LimeLight s_limelight;
    private Swerve s_Swerve;

    //ProfiledPIDController txPid;
    //ProfiledPIDController tyPid;
    //PIDController txPid;
    //PIDController tyPid;

    double KPX =.15;//.2
    double KPY =.15;//.2
    double KPO =.1;

    double tx;
    double ty;
    double omega;

    boolean teleop = false;
    boolean finished = false;
    Timer timer = new Timer();

    /* 
    DoubleSupplier translationSup = () -> tyPid.calculate(ty, 0);
    DoubleSupplier strafeSup = () -> txPid.calculate(tx, 0);
    DoubleSupplier rotationSup = () -> {return 0;};
    BooleanSupplier robotCentricSup = () -> {return false;};*/

    public AutoLimelightAlign(LimeLight s_limelight, Swerve s_Swerve) {
        this.s_limelight = s_limelight;
        this.s_Swerve = s_Swerve;
        //txPid = new ProfiledPIDController(txAlign.kp, txAlign.ki, txAlign.kd, txAlign.profile);
        //tyPid = new ProfiledPIDController(tyAlign.kp, tyAlign.ki, tyAlign.kd, tyAlign.profile);
        //txPid = new PIDController(txAlign.kp, txAlign.ki, txAlign.kd);
        //tyPid = new PIDController(tyAlign.kp, tyAlign.ki, tyAlign.kd);
       // txPid.setTolerance(txAlign.threshold);
        //tyPid.setTolerance(tyAlign.threshold);
    }

    private void fetchValues() {
       tx = s_limelight.getTX();
       ty = -s_limelight.getTY();
       omega = s_Swerve .getYaw().getDegrees();
    }

    @Override
    public void initialize() {
        if (DriverStation.isTeleopEnabled()){
            s_limelight.setPipeline(Constants.Limelight.Options.RetroReflective);
            teleop = true;
        }else{
            s_limelight.setPipeline(Constants.Limelight.Options.RetroReflectiveStand);
            teleop = false;
        }
        finished = false;
        timer.reset();
        timer.start();
        fetchValues();

        //txPid.reset(tx);
        //tyPid.reset(ty);

        //txPid.reset();
        //tyPid.reset();
    }

    @Override
    public void execute() {
       fetchValues();

       double Xerr = tx;
       double Yerr = ty;
       double Oerr  =-Conversions.wrap(s_Swerve.getYaw360(), 0); //angle 2 = desired 

       double rotationVal = Oerr / 90;



       
       //Translation2d translation = new Translation2d(txPid.calculate(-tx, 0), tyPid.calculate(-ty, 0));
       //Translation2d translation = new Translation2d(tyPid.calculate(-ty, 0), 0);
       Translation2d translation = new Translation2d(Yerr*(KPY), Xerr*(KPX));
       SmartDashboard.putNumber("txPid",  Xerr*(KPX));
       SmartDashboard.putNumber("tyPid", Yerr*(KPY));

        if (teleop && timer.hasElapsed(.2)){
            s_Swerve.drive(
                translation, rotationVal * Constants.Swerve.maxAngularVelocity, false, true
            );
        }else{
            s_Swerve.drive(
                translation, rotationVal * Constants.Swerve.maxAngularVelocity, false, true
            ); 
        }

        if (teleop && timer.hasElapsed(1.5)) finished =true;  


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
        return (Math.abs(tx) < txAlign.threshold) && (Math.abs(ty) < tyAlign.threshold) || finished;
    }

    @Override 
    public void end(boolean interupted) {
        s_Swerve.stop();
    }
}