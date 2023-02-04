package org.WaialuaRobotics359.robot.commands.autonomous;

import org.WaialuaRobotics359.robot.Robot;
import org.WaialuaRobotics359.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoBalance extends SequentialCommandGroup {

    private Swerve s_Swerve;    


    public AutoBalance(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

      addCommands(
          new RunCommand(
                  () -> {
                    s_Swerve.setModuleStates(
                        new SwerveModuleState[] {
                          new SwerveModuleState(-0.7, Rotation2d.fromDegrees(0)),
                          new SwerveModuleState(-0.7, Rotation2d.fromDegrees(0)),
                          new SwerveModuleState(-0.7, Rotation2d.fromDegrees(0)),
                          new SwerveModuleState(-0.7, Rotation2d.fromDegrees(0))
                        });
                  })
              .until(() -> s_Swerve.GetGyroPitch() < 10 && s_Swerve.GetGyroPitch()> 5),
          new RunCommand(
                  () -> {
                    s_Swerve.setModuleStates(
                        new SwerveModuleState[] {
                          new SwerveModuleState(0.2, Rotation2d.fromDegrees(0)),
                          new SwerveModuleState(0.2, Rotation2d.fromDegrees(0)),
                          new SwerveModuleState(0.2, Rotation2d.fromDegrees(0)),
                          new SwerveModuleState(0.2, Rotation2d.fromDegrees(0))
                        });
                  })
              .until(() -> s_Swerve.GetGyroPitch()< 10));
    }
  }