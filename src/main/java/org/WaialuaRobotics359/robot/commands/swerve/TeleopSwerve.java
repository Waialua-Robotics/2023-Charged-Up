package org.WaialuaRobotics359.robot.commands.swerve;

import org.WaialuaRobotics359.lib.math.Conversions;
import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    private double rotationalIncrement = 1;
    private Boolean feedbackNode = false;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.OI.deadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.OI.deadband);
        double omega = -MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.OI.deadband);

      /*   if (omega != 0 && !feedbackNode) {
            s_Swerve.desiredAngle = s_Swerve.getYaw().getDegrees();
            feedbackNode = true;
        } else if (omega == 0) {
            feedbackNode = false;
        }
        */
        s_Swerve.desiredAngle += omega * rotationalIncrement;
        s_Swerve.desiredAngle = (s_Swerve.desiredAngle + 360) % 360;

        SmartDashboard.putNumber("desired", s_Swerve.desiredAngle);
        SmartDashboard.putNumber("current", s_Swerve.getYaw360());
        
        double angleToDesired = Conversions.wrap(s_Swerve.getYaw360(), s_Swerve.desiredAngle);
        double rotationVal = angleToDesired / 180;

        SmartDashboard.putNumber("rotationval", rotationVal);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}