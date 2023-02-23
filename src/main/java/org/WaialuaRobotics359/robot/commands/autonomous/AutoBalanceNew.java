package org.WaialuaRobotics359.robot.commands.autonomous;

import org.WaialuaRobotics359.lib.math.Conversions;
import org.WaialuaRobotics359.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalanceNew extends CommandBase {
    private Swerve s_Swerve; 
    private Boolean forward;

    private double currentPitch;
    private double pitchOffset;
    private double pitchThreshold = 0.5;

    /* Variables for the drop case */
        //private double previousPitch; // Set to a value that will never be reached
        private int i;
        private int inRangeDuration;
    /* Variables for the balance case */ 
        private double dropAngle;

        private int iThreshold;
        private double brakeOverAngle;

    enum State {mount, drop, balance, finish}
    private State state = State.mount;

    public AutoBalanceNew(Swerve s_Swerve, Boolean forward) {
        this.s_Swerve = s_Swerve;
        this.forward = forward;
        addRequirements(s_Swerve);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pitchOffset = s_Swerve.GetGyroPitch();
        state = State.mount;

        /* Initialize variables to their default values */
        //previousPitch = 50; // Set to a value that will never be reached
        i = 0;
        inRangeDuration = 10;
        dropAngle = 14;

        iThreshold = 40;
        brakeOverAngle = 13.8;

    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        
        /*SmartDashboed Values */
            /*if(true){
            brakeOverAngle = SmartDashboard.getNumber("brakeOverAngle", 13.8);
            iThreshold = (int)SmartDashboard.getNumber("iThreshold",40);
        }*/

        currentPitch = s_Swerve.GetGyroPitch() - pitchOffset;

        switch (state) {
            case mount:

            //System.out.println("mount");

                double drivePower = (forward ? 2 : -2);

                s_Swerve.setModuleStates(
                    new SwerveModuleState[] {
                        new SwerveModuleState(drivePower, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(drivePower, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(drivePower, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(drivePower, Rotation2d.fromDegrees(0))
                    }
                );

                if (Math.abs(currentPitch) > 16) state = State.balance; //drop

                break;

            case drop:


            //System.out.println("drop");

                if (Conversions.isBetween(currentPitch, dropAngle - pitchThreshold, dropAngle + pitchThreshold)) {
                    i++;
                } else {
                    i = 0;
                }

                if (i > inRangeDuration ) {
                    state = State.balance;
                }

                break;  

            case balance:

            //System.out.println("balance");

                if ((Math.abs(currentPitch) <brakeOverAngle) && i >iThreshold) state = State.finish;

                i++;

                //if (Math.abs(currentPitch - dropAngle) > pitchThreshold) state = State.finish;

                break;

            case finish:
            break;
            
        }

        System.out.println(Math.abs(currentPitch));
        System.out.println(i);
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