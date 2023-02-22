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
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.util.HashMap;

import org.WaialuaRobotics359.robot.autos.*;
import org.WaialuaRobotics359.robot.autos.AutoCommandGroup.*;
import org.WaialuaRobotics359.robot.commands.AutoZero.*;
import org.WaialuaRobotics359.robot.commands.autonomous.*;
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

    public static boolean isCompetitionBot = false;

    public static boolean isCube = true;

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int ForkLiftTrigger = XboxController.Axis.kRightTrigger.value;
    private final int ForkLowerTrigger = XboxController.Axis.kLeftTrigger.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton ResetMods = new JoystickButton(driver, XboxController.Button.kBack.value); 
    private final JoystickButton Angle0 = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton Angle180 = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton setDriveSlowMode = new JoystickButton(driver, XboxController.Button.kRightBumper.value); 
    private final JoystickButton Angle90 = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton Angle270 = new JoystickButton(driver, XboxController.Button.kB.value);
    private final POVButton ForkDeploy = new POVButton(driver, 90);
    private final POVButton AutoZeroAll = new POVButton(driver, 180);
    private final JoystickButton setCurrentAngle = new JoystickButton(driver, XboxController.Button.kRightStick.value);
    private final POVButton AutoAlign = new POVButton(driver, 270);
    private final POVButton toggleCam = new POVButton(driver, 0);

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
    private final JoystickButton StowPosition = new JoystickButton(operator, Constants.OI.StowPosition);
    private final JoystickButton StandPosition = new JoystickButton(operator, Constants.OI.StandPosition);
    private final JoystickButton Intake = new JoystickButton(operator, Constants.OI.intake);
    private final JoystickButton Outake = new JoystickButton(operator, Constants.OI.outake);
    private final JoystickButton setCube = new JoystickButton(operator, Constants.OI.isCube);
    private final JoystickButton setCone = new JoystickButton(operator, Constants.OI.isCone);
    private final POVButton ZeroSlide = new POVButton(operator, Constants.OI.ZeroSlide);
    private final POVButton ZeroAll = new POVButton(operator, Constants.OI.ZeroAll);
    private final POVButton BirdPosition = new POVButton(operator, Constants.OI.BirdPosition);
    private final POVButton autoBalance = new POVButton(operator, 0);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Elevator s_Elevator = new Elevator();
    private final Slide s_Slide = new Slide();
    private final Wrist s_Wrist = new Wrist();
    private final Intake s_Intake = new Intake();
    private final LEDs s_LEDs = new LEDs();
    private final LimeLight s_LimeLight = new LimeLight();
    private final Fork s_Fork = new Fork();

    /* auto Builder */
    private SwerveAutoBuilder autoBuilder; 

    /* The autonomous routines */
    private final swerveBuilderAuto m_SwerveBuilderAuto;
    private final ConeL3Auto m_ConeL3Auto;
    private final ConeL1Auto m_ConeL1Auto;
    private final ConeL1Dual m_ConeL1Dual;
    private final ConeL1DualBalance m_ConeL1DualBalance;
    private final ConeM1Balance m_ConeM1Balance;
    private final CubeM2Balance m_CubeM2Balance;
    private final ConeM3Balance m_ConeM3Balance;
    private final ConeR3Balance m_ConeR3Balance;
    private final ConeM1ClearBalance m_ConeM1ClearBalance;
    private final CubeM2ClearBalance m_CubeM2ClearBalance;
    private final ConeM3ClearBalance m_ConeM3ClearBalance;
    private final ConeR3Dual m_ConeR3Dual;
    private final ConeR3DualBalance m_ConeR3DualBalance;
    private final ConeScoreHighStow m_ConeScoreHighStow;
    private final DriveBack m_DriveBack;
    private final DoNothing m_DoNothing;
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
                () -> operator.getRawAxis(WristAxisN),
                () -> operator.getRawAxis(WristAxis)
                )
        );

        CommandScheduler.getInstance().setDefaultCommand(s_Intake,
            new ManualIntake(
                s_Intake,
                () -> Intake.getAsBoolean(),
                () -> Outake.getAsBoolean()
                )
        );

        CommandScheduler.getInstance().setDefaultCommand(s_Fork,
            new ManualFork(
                s_Fork,
                () -> driver.getRawAxis(ForkLiftTrigger),
                () -> driver.getRawAxis(ForkLowerTrigger),
                () -> ForkDeploy.getAsBoolean()
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
        /*AutoBuilder */
        m_SwerveBuilderAuto = new swerveBuilderAuto(autoBuilder);
        m_ConeL3Auto = new ConeL3Auto(autoBuilder);
        m_ConeL1Auto = new ConeL1Auto(autoBuilder);
        m_ConeL1Dual = new ConeL1Dual(autoBuilder);
        m_ConeL1DualBalance = new ConeL1DualBalance(autoBuilder);
        m_ConeM1Balance = new ConeM1Balance(autoBuilder);
        m_CubeM2Balance = new CubeM2Balance(autoBuilder);
        m_ConeM3Balance = new ConeM3Balance(autoBuilder);
        m_ConeM1ClearBalance = new ConeM1ClearBalance(autoBuilder);
        m_CubeM2ClearBalance = new CubeM2ClearBalance(autoBuilder);
        m_ConeM3ClearBalance = new ConeM3ClearBalance(autoBuilder);
        m_ConeR3Balance = new ConeR3Balance(autoBuilder);
        m_ConeR3Dual = new ConeR3Dual(autoBuilder);
        m_ConeR3DualBalance = new ConeR3DualBalance(autoBuilder);
        m_DriveBack = new DriveBack(autoBuilder);
        /*Command */
        m_ConeScoreHighStow = new ConeScoreHighStow(s_Wrist, s_Elevator, s_Slide, s_Intake);
        m_DoNothing = new DoNothing();
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
            setCurrentAngle.onTrue(new InstantCommand(() -> s_Swerve.setCurrentAngle()));
            Angle0.onTrue(new InstantCommand(() -> s_Swerve.setDesired(180)));
            Angle90.onTrue(new InstantCommand(() -> s_Swerve.setDesired(270)));
            Angle180.onTrue(new InstantCommand(() -> s_Swerve.setDesired(0)));
            Angle270.onTrue(new InstantCommand(() -> s_Swerve.setDesired(90)));
            /*Misc Driver Binds */
            AutoZeroAll.onTrue(new AutoZeroAll(s_Wrist, s_Elevator, s_Slide));
            AutoAlign.onTrue(new AutoLimelightAlign(s_LimeLight, s_Swerve));
            toggleCam.onTrue( new InstantCommand(() -> s_LimeLight.toggleDriver()));
        /* Operator Buttons */
            /* Elevator System Positions */
            HighPosition.onTrue(new SetHighPosition(s_Wrist, s_Elevator, s_Slide));
            FeederPosition.onTrue(new SetFeederPosition(s_Wrist, s_Elevator, s_Slide));
            MidPosition.onTrue(new SetMidPosition(s_Wrist, s_Elevator, s_Slide));
            LowPosition.onTrue(new SetLowPosition(s_Wrist, s_Elevator, s_Slide));
            StowPosition.onTrue(new SetStowPosition(s_Wrist, s_Elevator, s_Slide));
            StandPosition.onTrue(new SetStandPosition(s_Wrist, s_Elevator, s_Slide));
            BirdPosition.onTrue(new SetBirdPosition(s_Wrist, s_Elevator, s_Slide));
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
            /*ZeroButtons */
            ZeroSlide.onTrue(new AutoZeroSlide(s_Slide));
            ZeroAll.onTrue(new AutoZeroAll(s_Wrist, s_Elevator, s_Slide));

            autoBalance.onTrue(new AutoBalanceNew(s_Swerve, false));

            /*DashboardCommand */
            SmartDashboard.putData("AutoBallance", new AutoBalance(s_Swerve));
            SmartDashboard.putData("AutoBallanceNewForward", new AutoBalanceNew(s_Swerve, true));
            SmartDashboard.putData("AutoBallanceNewReverse", new AutoBalanceNew(s_Swerve, false));
            SmartDashboard.putData("AutoIntakeConeSlide", new AutoIntakeConeSlide(s_Intake, s_Slide));
            SmartDashboard.putData("setElivatorToStart", new InstantCommand(()-> s_Elevator.SetPosition(66500)));
            SmartDashboard.putData("midFast", new MidScoreFast(s_Wrist, s_Elevator, s_Slide));
            //SmartDashboard.putData("AutoZeroslide", new AutoZeroSlide(s_Slide));
            //SmartDashboard.putData("AutoZeroElevator", new AutoZeroElevator(s_Elevator));
            //SmartDashboard.putData("AutoZeroWrist", new AutoZeroWrist(s_Wrist));
            //SmartDashboard.putData("AutoZeroAll", new AutoZeroAll(s_Wrist, s_Elevator, s_Slide));

            
    }

    public void configAuto() {
        /* Populate Sendable Chooser */
        m_chooser.setDefaultOption("swerveBuilderAuto",  "SwerveBuilderAuto");
        m_chooser.addOption("twomAuto", "twomAuto");   
        m_chooser.addOption("ConeL3Auto", "ConeL3Auto");
        m_chooser.addOption("ConeL1Auto", "ConeL1Auto");
        m_chooser.addOption("ConeL1Dual", "ConeL1Dual");
        m_chooser.addOption("ConeL1DualBalance", "ConeL1DualBalance");
        m_chooser.addOption("ConeM1Balance", "ConeM1Balance");
        m_chooser.addOption("CubeM2Balance", "CubeM2Balance");
        m_chooser.addOption("ConeM3Balance", "ConeM3Balance");
        m_chooser.addOption("ConeM1ClearBalance", "ConeM1ClearBalance");
        m_chooser.addOption("ConeM3ClearBalance", "ConeM3ClearBalance");
        m_chooser.addOption("CubeM2ClearBalance", "CubeM2ClearBalance");
        m_chooser.addOption("ConeR3Balance", "ConeR3Balance");
        m_chooser.addOption("ConeR3Dual", "ConeR3Dual");
        m_chooser.addOption("ConeR3DualBalance", "ConeR3DualBalance");
        m_chooser.addOption("DriveBack", "DriveBack");
        m_chooser.addOption("DoNothing", "DoNothing");

        m_chooser.addOption("ConeScoreHighStow", "ConeScoreHighStow");
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
        eventMap.put("IntakeConeTime", new AutoIntakeConeTime(s_Intake));

        eventMap.put("MidPosition",new SetMidPosition(s_Wrist, s_Elevator, s_Slide));
        eventMap.put("StowPosition",new SetStowPosition(s_Wrist, s_Elevator, s_Slide));
        eventMap.put("LowPosition", new SetLowPosition(s_Wrist, s_Elevator, s_Slide));
        eventMap.put("AutoBalance",new AutoBalance(s_Swerve));
        eventMap.put("AutoBalanceNewForward", new AutoBalanceNew(s_Swerve, true));
        eventMap.put("AutoBalanceNewBackward", new AutoBalanceNew(s_Swerve, false));
        eventMap.put("AutoBalanceForward",new AutoBalanceForward(s_Swerve));
        eventMap.put("OuttakeCone", new AutoOuttakeCone(s_Intake));
        eventMap.put("OuttakeCube", new AutoOuttakeCube(s_Intake));
        eventMap.put("StopSwerve", new InstantCommand( () -> s_Swerve.stop()));
        eventMap.put("StandPosition", new SetStandPosition(s_Wrist, s_Elevator, s_Slide));
        eventMap.put("MidScoreFast", new MidScoreFast(s_Wrist, s_Elevator, s_Slide));
        eventMap.put("ConeSlideIntake",new AutoIntakeConeSlide(s_Intake, s_Slide));
        eventMap.put("AutoBalanceInstant", new AutoBalanceInstant(s_Swerve));
        eventMap.put("AutoBalanceInstantForward", new AutoBalanceInstantForward(s_Swerve));

        eventMap.put("AutoLimelightAlign", new AutoLimelightAlign(s_LimeLight, s_Swerve));

        /*Comand Group */
        eventMap.put("ConeScoreMid",new ConeScoreMid(s_Wrist, s_Elevator, s_Slide, s_Intake));
        eventMap.put("ConeScoreHigh", new ConeScoreHigh(s_Wrist, s_Elevator, s_Slide, s_Intake));
        eventMap.put("ConeScoreHighStow", new ConeScoreHighStow(s_Wrist, s_Elevator, s_Slide, s_Intake));
        eventMap.put("CubeScoreHighStow", new CubeScoreHighStow(s_Wrist, s_Elevator, s_Slide, s_Intake));
        eventMap.put("ConeIntakeStow", new ConeIntakeStow(s_Wrist, s_Elevator, s_Slide, s_Intake));
        eventMap.put("ConeScoreHighHalf", new ConeScoreHighHalf(s_Wrist, s_Elevator, s_Slide, s_Intake));

        /*Wait Times */
        eventMap.put("Wait5", new AutoWait(5));
        eventMap.put("Wait1", new AutoWait(1));
        eventMap.put("Wait1.5",new AutoWait(1.5));


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

    public LEDs getLEDs(){
        return s_LEDs;
    }

    public LimeLight getLimelight(){
        return s_LimeLight;
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
            case "ConeL3Auto":
                selected = m_ConeL3Auto;
                break;
            case "ConeL1Auto":
                selected = m_ConeL1Auto;
                break;
            case "ConeL1Dual":
                selected = m_ConeL1Dual;
                break;
            case "ConeL1DualBalance":
                selected = m_ConeL1DualBalance;
                break;
            case "ConeM1Balance":
                selected = m_ConeM1Balance;
                break;
            case "CubeM2Balance":
                selected = m_CubeM2Balance;
                break;
            case "ConeM3Balance":
                selected = m_ConeM3Balance;
                break;
            case "ConeM1ClearBalance":
                selected = m_ConeM1ClearBalance;
                break;
            case "CubeM2ClearBalance":
                selected = m_CubeM2ClearBalance;
                break;
            case "ConeM3ClearBalance":
                selected = m_ConeM3ClearBalance;
                break;
            case "ConeR3Balance":
                selected = m_ConeR3Balance;
                break;
            case "ConeR3Dual":
                selected = m_ConeR3Dual;
                break;
            case "ConeR3DualBalance":
                selected = m_ConeR3DualBalance;
                break;
            case "ConeScoreHighStow":
                selected = m_ConeScoreHighStow;
                break;
            case "DriveBack":
                selected = m_DriveBack;
                break;
            case "DoNothing":
                selected = m_DoNothing;
                break;
            default:
                selected =  m_SwerveBuilderAuto;
                break;
        }

        return selected;
    }
}
