package org.WaialuaRobotics359.robot.commands.autonomous;

import org.WaialuaRobotics359.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoMountBalance extends CommandBase {
    private Swerve s_Swerve;
    
    public AutoMountBalance(Swerve s_Swerve, double currentAngle) {
      this.s_Swerve = s_Swerve;
      addRequirements(s_Swerve);
    }

    public void initialize(){}

    @Override
    public void execute(){
        
        s_Swerve.setModuleStates(
            new SwerveModuleState[] {
              new SwerveModuleState(-.7, Rotation2d.fromDegrees(0)),
              new SwerveModuleState(-.7, Rotation2d.fromDegrees(0)),
              new SwerveModuleState(-.7, Rotation2d.fromDegrees(0)),
              new SwerveModuleState(-.7, Rotation2d.fromDegrees(0))
            }
        );
    }

    @Override
  public void end(boolean interrupted) {
    //s_Swerve.stop();
  }
 
   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     return Math.abs(s_Swerve.GetGyroPitch()) >20; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
  }

}
