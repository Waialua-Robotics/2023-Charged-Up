
package org.WaialuaRobotics359.robot.commands.swerve;

import java.util.function.BooleanSupplier;

import org.WaialuaRobotics359.robot.subsystems.Swerve;
import org.WaialuaRobotics359.robot.util.LimelightHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoAlignXRetro extends CommandBase {

    private PoseEstimator s_PoseEstimator;
    private Swerve s_swerve;
    private BooleanSupplier alignButton;

    // Create a PID controller whose setpoint's change is subject to maximum
    // velocity and acceleration constraints.
    private static double period = .02; //0.02
    private final TrapezoidProfile.Constraints constraints;
    private final PIDController controller;

    private double xDistance;
    private Timer  Timer;

    public AutoAlignXRetro(PoseEstimator s_poseEstimator, Swerve s_swerve, BooleanSupplier alignButton) {
        this.s_PoseEstimator = s_poseEstimator;
        this.s_swerve = s_swerve;
        this.alignButton = alignButton;
        Timer = new Timer();
        constraints = new TrapezoidProfile.Constraints(1, .3);
        controller=  new PIDController(.08, .03, 0, period);
        controller.setTolerance(0.01);
    }

    private void fetchValues() {
       xDistance = -LimelightHelpers.getTX("limelight");
    }

    @Override
    public void initialize() {
        fetchValues();
        Timer.reset();
        Timer.start();
        controller.reset();
    }

    @Override
    public void execute() {
       fetchValues();
       
       Translation2d translation = new Translation2d(0, (controller.calculate(xDistance, 0))); // only drive x value
       SmartDashboard.putNumber("xDistanceRetro", controller.calculate(xDistance, 0));
       
        s_swerve.drive(
            translation, 0, false, true //#FIXME open loop? feild relitive? 
        ); 

        //System.out.println(translation.getY());
        //System.out.println(xDistance);

    }
    
    @Override
    public boolean isFinished(){        
        return (!alignButton.getAsBoolean());
    }

    @Override 
    public void end(boolean interupted) {
        s_swerve.stop();
    }
}