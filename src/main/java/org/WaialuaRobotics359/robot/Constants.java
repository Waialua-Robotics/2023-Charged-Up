package org.WaialuaRobotics359.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import org.WaialuaRobotics359.robot.commands.setPoints.SetLowElevatorPositionInches;

import java.util.HashMap;

import org.WaialuaRobotics359.lib.util.COTSFalconSwerveConstants;
import org.WaialuaRobotics359.lib.util.SwerveModuleConstants;

public final class Constants {
    public static HashMap<String, Command> eventMap = new HashMap<String, Command>();

    public static final class OI {
        public static final int driverPort = 0;
        public static final int operatorPort = 1;

        public static final int elevatorAxis = XboxController.Axis.kLeftY.value;
        public static final int slideAxis = XboxController.Axis.kLeftX.value;
        public static final int wristAxis = XboxController.Axis.kRightTrigger.value;
        public static final int WristAxisN = XboxController.Axis.kLeftTrigger.value;

        public static final int WristPositionDegrees = XboxController.Button.kX.value;
        public static final int LowElevatorPositionInches = XboxController.Button.kA.value;
        public static final int MidElevatorPositionInches = XboxController.Button.kB.value;
        public static final int HighElevatorPositionInches = XboxController.Button.kY.value;
        public static final int LowSlidePositionInches = XboxController.Button.kStart.value;
        public static final int HighSlidePositionInches = XboxController.Button.kBack.value;
        public static final int intake = XboxController.Button.kRightBumper.value;
        public static final int outake = XboxController.Button.kLeftBumper.value;

        public static final double deadband = 0.1;
    }

    public static final class LEDs {
        public static final int CANdleID = 0; //TODO: This must be tuned to robot

    }

    public static final class Slide {
        public static final int slideMotorID = 32;
        public static final TalonFXConfiguration config = new TalonFXConfiguration();
        public static final Boolean slideMotorInvert = false;
        public static final NeutralMode slideNeutralMode = NeutralMode.Brake;

        public static final float LowSlidePosition = 5;
        public static final float MidSlidePosition = 10;
        public static final float HighSlidePosition = 15;

        public static final float InchesPerRevolution = 12;

        public static final float Ratio = 15;

        public static final float TicsPerRevolution = (Ratio*2048);

        public static final int LowSlidePositionInches = (int) ((LowSlidePosition/InchesPerRevolution)*TicsPerRevolution);
        public static final int MidSlidePositionInches = (int) ((MidSlidePosition/InchesPerRevolution)*TicsPerRevolution);
        public static final int HighSlidePositionInches = (int) ((HighSlidePosition/InchesPerRevolution)*TicsPerRevolution);

        public static final int threshold = 15;

        public static final int LowPosition = 100;
        public static final int MidPosition = 50000;
        public static final int HighPosition = 90000;
        public static final int FeederPosition = 10000;

        /* soft limits */
        public static final int forwardSoftLimit = 205000; 
        public static final int reverseSoftLimit = 0;

        /* current limiting */
        public static final int continuousCurrentLimit = 25;
        public static final int peakCurrentLimit = 40;
        public static final double peakCurrentDuration = 0.1;
        public static final boolean enableCurrentLimit = true;

        /* PID */
        public static final double slideKP = 0.1; 
        public static final double slideKI = 0.0;
        public static final double slideKD = 0.0;
        public static final double slideKF = 0.0;
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
        public static final double closedLoopPeakOutput =.1;
    }

    public static final class Elevator {
        public static final int rElevatorID = 42;
        public static final int lElevatorID = 12;
        public static final TalonFXConfiguration config = new TalonFXConfiguration();
        public static final NeutralMode elevatorNeutralMode = NeutralMode.Brake;
        
        public static final float LowElevatorPosition = 5;
        public static final float MidElevatorPosition = 10;
        public static final float HighElevatorPosition = 15;

        public static final float InchesPerRevolution = 12;

        public static final float Ratio = 30;

        public static final float TicsPerRevolution = (Ratio*2048);

        public static final int LowElevatorPositionInches = (int) ((LowElevatorPosition/InchesPerRevolution)*TicsPerRevolution);
        public static final int MidElevatorPositionInches = (int) ((MidElevatorPosition/InchesPerRevolution)*TicsPerRevolution);
        public static final int HighElevatorPositionInches = (int) ((HighElevatorPosition/InchesPerRevolution)*TicsPerRevolution);

        public static final int threshold = 15;
        
        public static final int LowPosition = 100;
        public static final int MidPosition = 50000;
        public static final int HighPosition = 90000;
        public static final int FeederPosition = 10000;

        public static final int maxHeight = 295000;
        public static final int minHeight = 0;

        /* soft limits */
        public static final int forwardSoftLimit = 100000; 
        public static final int reverseSoftLimit = 0;

        /* current limiting */
        public static final int continuousCurrentLimit = 25;
        public static final int peakCurrentLimit = 40;
        public static final double peakCurrentDuration = 0.1;
        public static final boolean enableCurrentLimit = true;

        /* PID */
        public static final double slideKP = 0.1; 
        public static final double slideKI = 0.0;
        public static final double slideKD = 0.0;
        public static final double slideKF = 0.0;
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
        public static final double closedLoopPeakOutput =.1;
    }

    public static final class Wrist {
        public static final int wristID = 33;
        public static final TalonFXConfiguration cofig = new TalonFXConfiguration();
        public static final Boolean wristMotorInvert = false;
        public static final NeutralMode wristNeutralMode = NeutralMode.Brake;
        public static final float Wrist90Position = 0;

        public static final float DegreesPerRevolution = 360;

        public static final float Ratio = 20;

        public static final float TicsPerRevolution = (Ratio*2048);

        public static final int Wrist90PositionDegrees = (int) (Wrist90Position/DegreesPerRevolution*TicsPerRevolution);


        public static final int threshold = 15;

        public static final int LowPosition = 100;
        public static final int MidPosition = 50000;
        public static final int HighPosition = 90000;
        public static final int FeederPosition = 10000;

        /* soft limits */
        public static final int forwardSoftLimit = 11000; 
        public static final int reverseSoftLimit = 0;

        /* current limiting */
        public static final int continuousCurrentLimit = 25;
        public static final int peakCurrentLimit = 40;
        public static final double peakCurrentDuration = 0.1;
        public static final boolean enableCurrentLimit = true;

        /* PID */
        public static final double wristKP = 0.1; 
        public static final double wristKI = 0.0;
        public static final double wristKD = 0.0;
        public static final double wristKF = 0.0;
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
        public static final double closedLoopPeakOutput =.05;
    }

    public static final class Intake {
        public static final int intakeID = 22;
        public static final TalonFXConfiguration cofig = new TalonFXConfiguration();
        public static final Boolean intakeMotorInvert = false;
        public static final NeutralMode intakeNeutralMode = NeutralMode.Brake;

        public static final double speed = 0.5;

        /* current limiting */
        public static final int continuousCurrentLimit = 25;
        public static final int peakCurrentLimit = 40;
        public static final double peakCurrentDuration = 0.1;
        public static final boolean enableCurrentLimit = true;
    }

    public static final class Swerve {
        public static final int pigeonID = 0;
        public static final boolean invertGyro = false; // TODO: Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule = 
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(19.75); //TODO: This must be tuned to robot
        public static final double wheelBase = Units.inchesToMeters(19.75); //TODO: This must be tuned to robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.1; //TODO: This must be tuned to robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //FIXME: This must be tuned to robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10; //TODO: This must be tuned to robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */

         /*     Swerve Order
         *         0  1
         *         2  3
         */
        
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to robot
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(357.45);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(82.53);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to robot
            public static final int driveMotorID = 14;
            public static final int angleMotorID = 15;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(55.55);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to robot
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(268.24);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }



    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /*Auto Builder Const */
        public static final PIDConstants translationPID = new PIDConstants(0.5, 0, 0);
        public static final PIDConstants rotationPID = new PIDConstants(15, 0, 0); //d.05
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
