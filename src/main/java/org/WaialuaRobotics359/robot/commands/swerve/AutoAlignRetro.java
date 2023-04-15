
package org.WaialuaRobotics359.robot.commands.swerve;

import java.util.function.BooleanSupplier;

import org.WaialuaRobotics359.robot.RobotContainer;
import org.WaialuaRobotics359.robot.commands.setPoints.SetStowPosition;
import org.WaialuaRobotics359.robot.subsystems.Elevator;
import org.WaialuaRobotics359.robot.subsystems.LEDs;
import org.WaialuaRobotics359.robot.subsystems.Slide;
import org.WaialuaRobotics359.robot.subsystems.Swerve;
import org.WaialuaRobotics359.robot.subsystems.Wrist;
import org.WaialuaRobotics359.robot.subsystems.LEDs.State;
import org.WaialuaRobotics359.robot.util.LimelightHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoAlignRetro extends CommandBase {

    private Swerve s_swerve;
    private LEDs s_LEDs;
    private BooleanSupplier alignButton;

    // Create a PID controller whose setpoint's change is subject to maximum
    // velocity and acceleration constraints.
    private static double period = .02; //0.02
    private final PIDController xController;
    private final PIDController yController;

    private double xDistance;
    private double yDistance;
    private Timer  Timer;

    public AutoAlignRetro( Swerve s_swerve, BooleanSupplier alignButton , Wrist s_Wrist,Elevator s_Elevator, Slide s_Slide,LEDs s_LEDs) {
        this.s_swerve = s_swerve;
        this.s_LEDs = s_LEDs;
        this.alignButton = alignButton;
        Timer = new Timer();
        xController = new PIDController(.08, .03, 0, period);
        xController.setTolerance(0.01);
        yController = new PIDController(.08, .03, 0, period);
        yController.setTolerance(0.01);
    }

    private void fetchValues() {
       xDistance = LimelightHelpers.getTX("limelight");
       yDistance = -LimelightHelpers.getTY("limelight");
    }

    @Override
    public void initialize() {
        //new SetStowPosition(null, null, null);
        LimelightHelpers.setPipelineIndex("limelight", 2);
        fetchValues();
        Timer.reset();
        Timer.start();
        xController.reset();
        yController.reset();
        //s_LEDs.state= State.Green;
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getCurrentPipelineIndex("limelight") == 2) {
       fetchValues();
       
       Translation2d translation = new Translation2d((yController.calculate(yDistance, 0)), (xController.calculate(xDistance, 0)));
       SmartDashboard.putNumber("xDistanceRetro", xController.calculate(xDistance, 0));
       SmartDashboard.putNumber("yDistanceRetro", xController.calculate(yDistance, 0));
       
        s_swerve.drive(
            translation, 0, false, true //#FIXME open loop? feild relitive? 
        ); 

        //System.out.println(translation.getY());
        //System.out.println(xDistance);
        }

    }
    
    @Override
    public boolean isFinished(){        
        return (!alignButton.getAsBoolean());
    }

    @Override 
    public void end(boolean interupted) {
        s_swerve.stop();
        LimelightHelpers.setPipelineIndex("limelight", 0);
        if( RobotContainer.isCube){s_LEDs.state= State.purple;}else{s_LEDs.state= State.yellow;}
    }
}