package org.WaialuaRobotics359.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.HashMap;

import org.WaialuaRobotics359.robot.autos.*;
import org.WaialuaRobotics359.robot.commands.AutoZero.*;
import org.WaialuaRobotics359.robot.commands.autonomous.AutoBalance;
import org.WaialuaRobotics359.robot.commands.autonomous.AutoIntakeCone;
import org.WaialuaRobotics359.robot.commands.autonomous.AutoIntakeCube;
import org.WaialuaRobotics359.robot.commands.manual.*;
import org.WaialuaRobotics359.robot.commands.setPoints.*;
import org.WaialuaRobotics359.robot.commands.swerve.TeleopSwerve;
import org.WaialuaRobotics359.robot.subsystems.*;
import org.WaialuaRobotics359.robot.subsystems.LEDs.State;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    public static boolean isCube = true;

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton ResetMods = new JoystickButton(driver, XboxController.Button.kBack.value); 
    private final JoystickButton Angle0 = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton Angle180 = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton setDriveSlowMode = new JoystickButton(driver, XboxController.Button.kRightBumper.value); 
    private final JoystickButton intakeCube = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton intakeCone = new JoystickButton(driver, XboxController.Button.kB.value);

    /* Operator Controls */
    private final int elevatorAxis = Constants.OI.elevatorAxis;
    private final int SlideAxis = Constants.OI.slideAxis;
    private final int WristAxis = Constants.OI.wristAxis;
    private final int WristAxisN = Constants.OI.WristAxisN;

    /* Operator Buttons */
    private final JoystickButton HighPosition = new JoystickButton(operator, Constants.OI.HighPosition);
    private final JoystickButton FeederPosition = new JoystickButton(operator, Constants.OI.FeederPosition);
    private final JoystickButton MidPosition = new JoystickButton(operator, Constants.OI.MidPosition);
    private final JoystickButton LowPosition = new JoystickButton(operator, Constants.OI.LowPosition);
    private final JoystickButton StowPosition =new JoystickButton(operator, Constants.OI.StowPosition);
    private final JoystickButton Intake = new JoystickButton(operator, Constants.OI.intake);
    private final JoystickButton Outake = new JoystickButton(operator, Constants.OI.outake);
    private final JoystickButton setCube = new JoystickButton(operator, Constants.OI.isCube);
    private final JoystickButton setCone = new JoystickButton(operator, Constants.OI.isCone);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Elevator s_Elevator = new Elevator();
    private final Slide s_Slide = new Slide();
    private final Wrist s_Wrist = new Wrist();
    private final Intake s_Intake = new Intake();
    private final LEDs s_LEDs = new LEDs();

    /* auto Builder */
    private SwerveAutoBuilder autoBuilder; 

    /* The autonomous routines */
    private final swerveBuilderAuto m_SwerveBuilderAuto;

    /* chooser for autonomous commands */
    SendableChooser<String> m_chooser = new SendableChooser<>();

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

        /* Configure Button Bindings */
        configureButtonBindings();

        /* Configure Autonomous Assets */
        configAuto();

        /** 
         * Initialize Autonomous Routines 
         * Do not move to prevent initialization race case
         */ 
        m_SwerveBuilderAuto = new swerveBuilderAuto(autoBuilder);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
            /* Reset Gyro */
            zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
            /* Reset Swerve Modules */
            ResetMods.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
            /* Snap-to Swerve Angle */
            Angle0.onTrue(new InstantCommand(() -> s_Swerve.setDesired(0)));
            Angle180.onTrue(new InstantCommand(() -> s_Swerve.setDesired(180)));
            intakeCube.onTrue(new AutoIntakeCube(s_Intake));
            intakeCone.onTrue(new AutoIntakeCone(s_Intake));

        /* Operator Buttons */
            /* Elevator System Positions */
            HighPosition.onTrue(new SetHighPosition(s_Wrist, s_Elevator, s_Slide));
            FeederPosition.onTrue(new SetFeederPosition(s_Wrist, s_Elevator, s_Slide));
            MidPosition.onTrue(new SetMidPosition(s_Wrist, s_Elevator, s_Slide));
            LowPosition.onTrue(new SetLowPosition(s_Wrist, s_Elevator, s_Slide));
            StowPosition.onTrue(new SetStowPosition(s_Wrist, s_Elevator, s_Slide));
            /* Toggle Game Piece */
            setCube.onTrue(
                new ParallelCommandGroup( new InstantCommand(() -> isCube = true),
                new InstantCommand(() -> s_LEDs.state= State.purple)));
            setCone.onTrue(
                new ParallelCommandGroup( new InstantCommand(() -> isCube = false),
                new InstantCommand(() -> s_LEDs.state= State.yellow)));
            /* Toggle Swerve Slow Mode */
            setDriveSlowMode.onTrue(new InstantCommand(()-> s_Swerve.slowMode =true ));
            setDriveSlowMode.onFalse(new InstantCommand(() -> s_Swerve.slowMode = false));

            /*DashboardCommand */
            SmartDashboard.putData("AutoBallance", new AutoBalance(s_Swerve));
            SmartDashboard.putData("AutoZeroslide", new AutoZeroSlide(s_Slide));
            SmartDashboard.putData("AutoZeroElevator", new AutoZeroElevator(s_Elevator));
            SmartDashboard.putData("AutoZeroWrist", new AutoZeroWrist(s_Wrist));
            SmartDashboard.putData("AutoZeroAll", new AutoZeroAll(s_Wrist, s_Elevator, s_Slide));

            
    }

    public void configAuto() {
        /* Populate Sendable Chooser */
        m_chooser.setDefaultOption("swerveBuilderAuto",  "SwerveBuilderAuto");
        m_chooser.addOption("twomAuto", "twomAuto");   
        Shuffleboard.getTab("Autonomous").add(m_chooser);

        /* Populate Event Map */
        HashMap<String, Command> eventMap = new HashMap<String, Command>();
        eventMap.put("LedYellow", new InstantCommand(() -> s_LEDs.state = State.yellow));
        eventMap.put("LedPurple", new InstantCommand(() -> s_LEDs.state = State.purple));
        eventMap.put("SetCube", new InstantCommand(() -> isCube = true));
        eventMap.put("SetCone", new InstantCommand(()-> isCube = false));
        eventMap.put("IntakeCone", new AutoIntakeCone(s_Intake));
        eventMap.put("IntakeCube", new AutoIntakeCube(s_Intake));
        eventMap.put("IntakeStop", new InstantCommand(()-> s_Intake.stop()));
        eventMap.put("MidPosition",new SetMidPosition(s_Wrist, s_Elevator, s_Slide));
        eventMap.put("StowPosition",new SetStowPosition(s_Wrist, s_Elevator, s_Slide));
        eventMap.put("LowPosition", new SetLowPosition(s_Wrist, s_Elevator, s_Slide));
        eventMap.put("AutoBalance",new AutoBalance(s_Swerve));

        /* Auto Builder */
        autoBuilder = new SwerveAutoBuilder(
            s_Swerve::getPose,
            s_Swerve::resetOdometry,
            Constants.Swerve.swerveKinematics,
            new PIDConstants(Constants.AutoConstants.translationPID.kP, Constants.AutoConstants.translationPID.kI,
                Constants.AutoConstants.translationPID.kD),
            new PIDConstants(Constants.AutoConstants.rotationPID.kP, Constants.AutoConstants.rotationPID.kI,
                Constants.AutoConstants.rotationPID.kD),
            s_Swerve::setModuleStates,
            eventMap,
            s_Swerve
        );
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

    public Swerve getSwerve(){
        return s_Swerve;
    }

    public Intake getIntake(){
        return s_Intake;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        Command selected;

        switch (m_chooser.getSelected()) {
            case "swerveBuilderAuto":
                selected = m_SwerveBuilderAuto;
                break;
            case "twomAuto":
                selected = m_SwerveBuilderAuto;
                break;
            default:
                selected =  m_SwerveBuilderAuto;
                break;
        }

        return selected;
    }
}
