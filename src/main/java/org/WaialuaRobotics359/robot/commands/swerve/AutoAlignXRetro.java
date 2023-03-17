
package org.WaialuaRobotics359.robot.commands.swerve;

import java.lang.ModuleLayer.Controller;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.LimeLight;
import org.WaialuaRobotics359.robot.subsystems.Swerve;
import org.WaialuaRobotics359.robot.util.LimelightHelpers;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoAlignXRetro extends CommandBase {

    private PoseEstimator s_PoseEstimator;
    private Swerve s_swerve;

    // Create a PID controller whose setpoint's change is subject to maximum
    // velocity and acceleration constraints.
    private static double period = .02; //0.02
    private final TrapezoidProfile.Constraints constraints;
    private final ProfiledPIDController controller;

    private double xDistance;
    private Timer  Timer;

    public AutoAlignXRetro(PoseEstimator s_poseEstimator, Swerve s_swerve) {
        this.s_PoseEstimator = s_poseEstimator;
        this.s_swerve = s_swerve;
        Timer = new Timer();
        constraints = new TrapezoidProfile.Constraints(1, 0.5);
        controller=  new ProfiledPIDController(.15, 0.0, 0, constraints, period);
        controller.setTolerance(0.5);
    }

    private void fetchValues() {
       xDistance = LimelightHelpers.getTX("limelight");
    }

    @Override
    public void initialize() {
        fetchValues();
        Timer.reset();
        Timer.start();
        controller.reset(xDistance);
    }

    @Override
    public void execute() {
       fetchValues();
       
       Translation2d translation = new Translation2d(controller.calculate(xDistance, 0), 0); // only drive x value
       SmartDashboard.putNumber("xDistanceRetro", controller.calculate(xDistance, 0));
       
        s_swerve.drive(
            translation, 0, true, true //#FIXME open loop? feild relitive? 
        ); 

    }
    
    @Override
    public boolean isFinished(){        
        return (controller.atSetpoint() || Timer.hasElapsed(2));
    }

    @Override 
    public void end(boolean interupted) {
        s_swerve.stop();
    }
}