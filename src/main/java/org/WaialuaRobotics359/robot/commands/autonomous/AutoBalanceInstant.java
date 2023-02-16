package org.WaialuaRobotics359.robot.commands.autonomous;

import org.WaialuaRobotics359.lib.math.Conversions;
import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Swerve;

import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalanceInstant extends CommandBase {
  private Swerve s_Swerve;    
  private Timer Timer = new Timer();

  private boolean balancing = false;
  private boolean finished = false;

  private double error;
  private double currentAngle;
  private double drivePower;
  private double pitchOffset;
  private double balanceAngle = 0;

  private int i = 0;
  
  public AutoBalanceInstant(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
  }

  public void initialize(){
    balanceAngle = 0;
    balancing = false;
    finished = false;
    pitchOffset = s_Swerve.GetGyroPitch();
    Timer.reset();
  }

  @Override
  public void execute() {

    // Uncomment the line below this to simulate the gyroscope axis with a controller joystick
    // Double currentAngle = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
    currentAngle = s_Swerve.GetGyroPitch() - pitchOffset;

    error = Constants.AutoConstants.BalanceGoal - currentAngle;

    // if first time, use second condition. if second time, use first condition
    if (Math.abs(error) < (balancing ? Constants.AutoConstants.BalanceThreshold : 20) ) {

        /* 
         * if the balancing sequence has begun and the pitch is in range,
         * the robot has balanced on the platform. return from the execute
         * and initiate the ending sequence.
         */

        if(!balancing){
          s_Swerve.setModuleStates(
            new SwerveModuleState[] {
              new SwerveModuleState(-2, Rotation2d.fromDegrees(0)),
              new SwerveModuleState(-2, Rotation2d.fromDegrees(0)),
              new SwerveModuleState(-2, Rotation2d.fromDegrees(0)),
              new SwerveModuleState(-2, Rotation2d.fromDegrees(0))
            }
        );
        }

    } else {

        // indicates that the balancing sequence has begun
        balancing = true;

      Timer.start();

      if (Timer.hasElapsed(1.2))
        if (i<1){
          i++;
          balanceAngle = error;
        }

        if (balanceAngle!= 0){
          finished = !Conversions.isBetween(error, (balanceAngle -.5), (balanceAngle+.5));
        }

        drivePower = -Math.min(.2 * error, 1); //.03 const

        // Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
        if (drivePower > 0) {
          drivePower *= 1;
        }

        s_Swerve.setModuleStates(
                  new SwerveModuleState[] {
                    new SwerveModuleState(drivePower, Rotation2d.fromDegrees(0)),
                    new SwerveModuleState(drivePower, Rotation2d.fromDegrees(0)),
                    new SwerveModuleState(drivePower, Rotation2d.fromDegrees(0)),
                    new SwerveModuleState(drivePower, Rotation2d.fromDegrees(0))
                  }
        );

    }

    // Debugging Print Statments
    //System.out.println("balance angle " + balanceAngle);
    //System.out.println("moved " + !Conversions.isBetween(error, (balanceAngle -1), (balanceAngle+1)));
    //System.out.println("Drive Power: " + drivePower);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.stop();
  }
 
   // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (finished); // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
  }
}