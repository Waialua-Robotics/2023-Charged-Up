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

import org.WaialuaRobotics359.robot.autos.*;
import org.WaialuaRobotics359.robot.commands.manual.*;
import org.WaialuaRobotics359.robot.commands.setPoints.*;
import org.WaialuaRobotics359.robot.commands.swerve.TeleopSwerve;
import org.WaialuaRobotics359.robot.subsystems.*;

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
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton ResetMods = new JoystickButton(driver, XboxController.Button.kStart.value); 

    /* Operator Controls */
    private final int elevatorAxis = Constants.OI.elevatorAxis;
    private final int SlideAxis = Constants.OI.slideAxis;
    private final int WristAxis = Constants.OI.wristAxis;
    private final int WristAxisN = Constants.OI.WristAxisN;

    /* Operator Buttons */
    private final JoystickButton HighPosition = new JoystickButton(operator, Constants.OI.HighPosition);
    private final JoystickButton MidPosition = new JoystickButton(operator, Constants.OI.MidPosition);
    private final JoystickButton LowPosition = new JoystickButton(operator, Constants.OI.LowPosition);
    private final JoystickButton Intake = new JoystickButton(operator, Constants.OI.intake);
    private final JoystickButton Outake = new JoystickButton(operator, Constants.OI.outake);
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Elevator s_Elevator = new Elevator();
    private final Slide s_Slide = new Slide();
    private final Wrist s_Wrist = new Wrist();
    private final Intake s_Intake = new Intake();

    /*Create Auto Builder*/ //#TODO: fix this in the future currently dosent work 
    public final SwerveAutoBuilder s_SwerveAutoBuilder = new SwerveAutoBuilder(s_Swerve::getPose,
    s_Swerve::resetOdometry,
    Constants.Swerve.swerveKinematics,
    new PIDConstants(Constants.AutoConstants.translationPID.kP, Constants.AutoConstants.translationPID.kI,
        Constants.AutoConstants.translationPID.kD),
    new PIDConstants(Constants.AutoConstants.rotationPID.kP, Constants.AutoConstants.rotationPID.kI,
        Constants.AutoConstants.rotationPID.kD),
        s_Swerve::setModuleStates,
    Constants.eventMap,
    s_Swerve);

    //private final LEDsSubsystem s_LEDs = new LEDsSubsystem();
 
    /*The autonomous routines*/

    private final Command m_twomAuto = new twomAuto(s_Swerve);
    private final Command m_SwerveBuilderAuto = new swerveBuilderAuto(s_Swerve);

    /*chooser for autonomous commands*/
    SendableChooser<Command> m_chooser = new SendableChooser<>();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
      public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
            s_Swerve, 
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis), 
            () -> -driver.getRawAxis(rotationAxis), 
            () -> robotCentric.getAsBoolean()
            )
        );  

        /* invert y axis */
        CommandScheduler.getInstance().setDefaultCommand(s_Elevator,
            new ManualElevator(
                s_Elevator,
                () -> -operator.getRawAxis(elevatorAxis)
                )
        );

        /* invert y axis */
        CommandScheduler.getInstance().setDefaultCommand(s_Slide,
            new ManualSlide(
                s_Slide,
                () -> -operator.getRawAxis(SlideAxis)
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

        // Add commands to the autonomous command chooser
        m_chooser.setDefaultOption("swerveBuilderAuto", m_SwerveBuilderAuto);
        m_chooser.addOption("twomAuto", m_twomAuto);                   // Put the chooser on the dashboard
        Shuffleboard.getTab("Autonomous").add(m_chooser);

        // Populate the autonomous event map
        setEventMap();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        ResetMods.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
        HighPosition.onTrue(new SetHighPosition(s_Wrist, s_Elevator, s_Slide));
        MidPosition.onTrue(new SetMidPosition(s_Wrist, s_Elevator, s_Slide));
        LowPosition.onTrue(new SetLowPosition(s_Wrist, s_Elevator, s_Slide));
    }

    public void setEventMap() {
        //Constants.eventMap.put("LedBlue", new InstantCommand(() -> s_LEDs.LEDsBlue()));
        //Constants.eventMap.put("intakeRetract", new InstantCommand(m_robotIntake::intakeRetract, m_robotIntake));
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
    public Command getAutonomousCommand() {
        //selected auto will run
       return m_chooser.getSelected();
    }
}
