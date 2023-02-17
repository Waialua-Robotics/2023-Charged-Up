package org.WaialuaRobotics359.robot.commands.autonomous;

import org.WaialuaRobotics359.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalance2 extends CommandBase {
    private Swerve s_Swerve; 
    private Boolean forward;

    private double currentPitch;
    private double pitchOffset;
    private double pitchThreshold = 0.5;

    /* Variables for the drop case */
        private double previousPitch = 50; // Set to a value that will never be reached
        private int i = 0;
        private int inRangeDuration = 10;
    /* Variables for the balance case */ 
        private double dropAngle;

    enum State {mount, drop, balance, finish}
    private State state = State.mount;

    public AutoBalance2(Swerve s_Swerve, Boolean forward) {
        this.s_Swerve = s_Swerve;
        this.forward = forward;
        addRequirements(s_Swerve);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pitchOffset = s_Swerve.GetGyroPitch();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        currentPitch = s_Swerve.GetGyroPitch() - pitchOffset;

        switch (state) {
            case mount:

                double drivePower = (forward ? 0.7 : - 0.7);

                s_Swerve.setModuleStates(
                    new SwerveModuleState[] {
                        new SwerveModuleState(drivePower, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(drivePower, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(drivePower, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(drivePower, Rotation2d.fromDegrees(0))
                    }
                );

                if (currentPitch > 20) state = State.drop;

                break;

            case drop:

                if (Math.abs(currentPitch - previousPitch) < pitchThreshold) {
                    i++; 
                } else {
                    i = 0;
                }

                previousPitch = currentPitch;

                if (i > inRangeDuration) {
                    state = State.balance;
                    dropAngle = currentPitch;
                }

                break;  

            case balance:

                if (Math.abs(currentPitch - dropAngle) > pitchThreshold) state = State.finish;

                break;
            
        }
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        s_Swerve.stop();
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (state == State.finish);
    }
    
}