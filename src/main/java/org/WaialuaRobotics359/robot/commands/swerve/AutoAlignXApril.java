
package org.WaialuaRobotics359.robot.commands.swerve;

import java.util.function.BooleanSupplier;

import org.WaialuaRobotics359.robot.RobotContainer;
import org.WaialuaRobotics359.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoAlignXApril extends CommandBase {

    private PoseEstimator s_PoseEstimator;
    private Swerve s_swerve;
    private BooleanSupplier alignButton;

    // Create a PID controller whose setpoint's change is subject to maximum
    // velocity and acceleration constraints.
    private static double kDt = 0.02;
    private final TrapezoidProfile.Constraints constraints;
    private final ProfiledPIDController controller;

    private double xDistance;
    private Timer  Timer;

    public AutoAlignXApril(PoseEstimator s_poseEstimator, Swerve s_swerve, BooleanSupplier alignButton ) {
        this.s_PoseEstimator = s_poseEstimator;
        this.s_swerve = s_swerve;
        this.alignButton = alignButton;
        Timer = new Timer();
        constraints = new TrapezoidProfile.Constraints(2, 1);
        controller = RobotContainer.isCube ? new ProfiledPIDController(2, 0.5, 0, constraints, kDt) : new ProfiledPIDController(2.5, 1, 0, constraints, kDt) ;
        controller.setTolerance(0.01);
    }

    private void fetchValues() {
       xDistance = s_PoseEstimator.getXtoClosestSelectedNode();
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
       
       Translation2d translation = new Translation2d(0, controller.calculate(xDistance, 0) +.22); // only drive y value
       SmartDashboard.putNumber("xDistance", controller.calculate(xDistance, 0));
       
        s_swerve.drive(
            translation, 0, true, true //open loop?
        ); 

        //System.out.println(translation.getY());
    }
    
    @Override
    public boolean isFinished(){        
        return (!alignButton.getAsBoolean()); //Timer.hasElapsed(2) || controller.atGoal() && !alignButton.getAsBoolean()
    }

    @Override 
    public void end(boolean interupted) {
        s_swerve.stop();
    }
}