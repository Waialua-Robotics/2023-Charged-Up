package org.WaialuaRobotics359.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.sql.Driver;

import org.WaialuaRobotics359.robot.autos.*;
import org.WaialuaRobotics359.robot.commands.manual.ManualElevator;
import org.WaialuaRobotics359.robot.commands.manual.ManualIntake;
import org.WaialuaRobotics359.robot.commands.manual.ManualSlide;
import org.WaialuaRobotics359.robot.commands.manual.ManualWrist;
import org.WaialuaRobotics359.robot.commands.setPoints.SetLowElevatorPositionInches;
import org.WaialuaRobotics359.robot.commands.setPoints.SetMidElevatorPositionInches;
import org.WaialuaRobotics359.robot.commands.setPoints.SetWrist90PositionDegrees;
import org.WaialuaRobotics359.robot.commands.setPoints.SetHighElevatorPositionInches;
import org.WaialuaRobotics359.robot.commands.setPoints.SetLowSlidePositionInches;
import org.WaialuaRobotics359.robot.commands.setPoints.SetHighSlidePositionInches;
import org.WaialuaRobotics359.robot.subsystems.*;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    //#FIXME //private final int translationAxis = XboxController.Axis.kLeftY.value;
    //#FIXME //private final int strafeAxis = XboxController.Axis.kLeftX.value;
    //#FIXME //private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
   //#FIXME // private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    //#FIXME //private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    //#FIXME //private final JoystickButton ResetMods = new JoystickButton(driver, XboxController.Button.kStart.value); 

    /* Operator Controls */
    private final int elevatorAxis = Constants.OI.elevatorAxis;
    private final int SlideAxis = Constants.OI.slideAxis;
    private final int WristAxis = Constants.OI.wristAxis;
    private final int WristAxisN = Constants.OI.WristAxisN;

    /* Operator Buttons */
    private final JoystickButton Intake = new JoystickButton(operator, Constants.OI.intake);
    private final JoystickButton Outake = new JoystickButton(operator, Constants.OI.outake);
    private final JoystickButton LowElevatorPositionInches = new JoystickButton(operator, Constants.OI.LowElevatorPositionInches);
    private final JoystickButton MidElevatorPositionInches = new JoystickButton(operator, Constants.OI.MidElevatorPositionInches);
    private final JoystickButton HighElevatorPositionInches = new JoystickButton(operator, Constants.OI.HighElevatorPositionInches);
    private final JoystickButton LowSlidePositionInches = new JoystickButton(operator, Constants.OI.LowSlidePositionInches);
    private final JoystickButton HighSlidePositionInches = new JoystickButton(operator, Constants.OI.HighSlidePositionInches);
    private final JoystickButton Wrist90PositionDegrees = new JoystickButton(operator, Constants.OI.WristPositionDegrees);
    /* Subsystems */
    //#FIXME //private final Swerve s_Swerve = new Swerve();
    private final Elevator s_Elevator = new Elevator();
    private final Slide s_Slide = new Slide();
    private final Wrist s_Wrist = new Wrist();
    private final Intake s_Intake = new Intake();

    //private final LEDsSubsystem s_LEDs = new LEDsSubsystem();
    
    /*The autonomous routines*/
    
    //#FIXME //private final Command m_twomAuto = new twomAuto(s_Swerve);
    //#FIXME //private final Command m_SwerveBuilderAuto = new swerveBuilderAuto(s_Swerve);

    /*chooser for autonomous commands*/
    //#FIXME //SendableChooser<Command> m_chooser = new SendableChooser<>();

   
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    //#FIXME 
      public RobotContainer() {
        /* 
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
            s_Swerve, 
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis), 
            () -> -driver.getRawAxis(rotationAxis), 
            () -> robotCentric.getAsBoolean()
            )
        );  
        */

        CommandScheduler.getInstance().setDefaultCommand(s_Elevator,
            new ManualElevator(
                s_Elevator,
                () -> operator.getRawAxis(elevatorAxis)
                )
        );

        CommandScheduler.getInstance().setDefaultCommand(s_Slide,
            new ManualSlide(
                s_Slide,
                () -> operator.getRawAxis(SlideAxis)
                )
        );

        CommandScheduler.getInstance().setDefaultCommand(s_Wrist,
            new ManualWrist(
                s_Wrist,
                () -> operator.getRawAxis(WristAxis),
                () -> operator.getRawAxis(WristAxisN)
                )
        );

        CommandScheduler.getInstance().setDefaultCommand(s_Intake,
            new ManualIntake(
                s_Intake,
                () -> Intake.getAsBoolean(),
                () -> Outake.getAsBoolean()
                )
        );

        // Configure the button bindings
        configureButtonBindings();

        /*#FIXME 
        // Create Auto Builder
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            s_Swerve::getPose,
            s_Swerve::resetOdometry,
            Constants.Swerve.swerveKinematics,
            new PIDConstants(Constants.AutoConstants.translationPID.kP, Constants.AutoConstants.translationPID.kI,
                Constants.AutoConstants.translationPID.kD),
            new PIDConstants(Constants.AutoConstants.rotationPID.kP, Constants.AutoConstants.rotationPID.kI,
                Constants.AutoConstants.rotationPID.kD),
                s_Swerve::setModuleStates,
            Constants.eventMap,
            s_Swerve
            );

            */
        // Add commands to the autonomous command chooser
        //#FIXME //m_chooser.setDefaultOption("swerveBuilderAuto", m_SwerveBuilderAuto);
        //#FIXME //m_chooser.addOption("twomAuto", m_twomAuto);

        // Put the chooser on the dashboard
        //#FIXME //Shuffleboard.getTab("Autonomous").add(m_chooser);

        // Populate the autonomous event map
        //#FIXME //setEventMap();
    }
    public static void SetElevatorPositionInches (double desired){
        double position = (desired*61440)/(Constants.Elevator.Ratio);
        SetElevatorPositionInches(position);
        Elevator.mElevatorMotorR.set(TalonFXControlMode.Position, position);
     }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
       //#FIXME // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        //#FIXME //ResetMods.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
        LowElevatorPositionInches.onTrue(new SetLowElevatorPositionInches(s_Elevator));
        MidElevatorPositionInches.onTrue(new SetMidElevatorPositionInches(s_Elevator));
        HighElevatorPositionInches.onTrue(new SetHighElevatorPositionInches(s_Elevator));
        LowSlidePositionInches.onTrue(new SetLowSlidePositionInches(s_Slide));
        HighSlidePositionInches.onTrue(new SetHighSlidePositionInches(s_Slide));
        Wrist90PositionDegrees.onTrue(new SetWrist90PositionDegrees(s_Wrist));
    }

    public void setEventMap() {
    //Constants.eventMap.put("LedBlue", new InstantCommand(() -> s_LEDs.LEDsBlue()));
    //SystemConstants.eventMap.put("intakeRetract", new InstantCommand(m_robotIntake::intakeRetract, m_robotIntake));
      }

      public Elevator getElevator() {
        return s_Elevator;
    }

    public Slide getSlide() {
        return s_Slide;
    }

    public Wrist getWrist() {
        return s_Wrist;
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    //public Command getAutonomousCommand() {
        // selected auto will run
       //#FIXME // return m_chooser.getSelected();
   // }
}
