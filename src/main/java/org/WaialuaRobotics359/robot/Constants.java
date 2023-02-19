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

import org.WaialuaRobotics359.lib.util.COTSFalconSwerveConstants;
import org.WaialuaRobotics359.lib.util.SwerveModuleConstants;

public final class Constants {

    public static final boolean isCompetitionRobot = false; //#TODO: Change to true for competition Bot

    public static final class OI {
        public static final int driverPort = 0;
        public static final int operatorPort = 1;

        public static final int elevatorAxis = XboxController.Axis.kLeftY.value;
        public static final int slideAxis = XboxController.Axis.kRightY.value;
        public static final int wristAxis = XboxController.Axis.kRightTrigger.value;
        public static final int WristAxisN = XboxController.Axis.kLeftTrigger.value;

        public static final int HighPosition = XboxController.Button.kY.value;
        public static final int FeederPosition =XboxController.Button.kX.value;
        public static final int MidPosition = XboxController.Button.kB.value;
        public static final int LowPosition = XboxController.Button.kA.value;
        public static final int StowPosition = XboxController.Button.kBack.value;
        public static final int StandPosition = XboxController.Button.kStart.value;
        public static final int intake = XboxController.Button.kRightBumper.value;
        public static final int outake = XboxController.Button.kLeftBumper.value;
        public static final int isCube = XboxController.Button.kLeftStick.value;
        public static final int isCone = XboxController.Button.kRightStick.value;
        public static final int ZeroSlide = 270;
        public static final int ZeroAll = 180;

        public static final double deadband = 0.1;
    }

    public static final class LEDs {
        public static final int CANdleID = 0; 
        public static final int LEDCount = 128; 

    }


    public static final class Fork {

        public static final int forkMotorID = 50;
        public static final TalonFXConfiguration config = new TalonFXConfiguration();

        public static final Boolean forkMotorInvert = false;
        public static final NeutralMode forkNeutralMode = NeutralMode.Brake;

    }
    
    public static final class Slide {
        public static final int slideMotorID = 20; 
        public static final TalonFXConfiguration config = new TalonFXConfiguration();
        public static final Boolean slideMotorInvert = false;
        public static final NeutralMode slideNeutralMode = NeutralMode.Brake;

        public static final int Ratio = 15;

        public static final int threshold = 500;

        public static final class Cube {
            public static final int LowPosition = 3000;
            public static final int MidPosition = 4600;
            public static final int HighPosition = 200000;
            public static final int FeederPosition = 1400;
            public static final int standPosition = 4300;
        }

        public static final class Cone {
            public static final int LowPosition = 3000;
            public static final int MidPosition = 85000;
            public static final int HighPosition = 204000;
            public static final int FeederPosition = 9500;
            public static final int standPosition = 4300;
        }
       
        /* soft limits */
        public static final int forwardSoftLimit = 208000; 
        public static final int reverseSoftLimit = 1000;

        /* current limiting */
        public static final int continuousCurrentLimit = 25;
        public static final int peakCurrentLimit = 40;
        public static final double peakCurrentDuration = 0.1;
        public static final boolean enableCurrentLimit = true;

        /*Motion Magic */
        public static final int velocity = 40000;
        public static final int acceleration = 40000;
        public static final int smoothing = 0; // 0-8

        /* PID */
        public static final double slideKP = 0.1; 
        public static final double slideKI = 0.0;
        public static final double slideKD = 0.0;
        public static final double slideKF = 0.0;
        //public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
        public static final double closedLoopPeakOutput = 1;
    }

    public static final class Elevator {
        public static final int rElevatorID = 10; 
        public static final int lElevatorID = 11; 
        public static final TalonFXConfiguration config = new TalonFXConfiguration();
        public static final NeutralMode elevatorNeutralMode = NeutralMode.Brake;

        public static final int MagElevatorID = 0;
        
        public static final int Ratio = 30;

        public static final int threshold = 500;

        public static final class Cube {
            public static final int LowPosition = 3000;
            public static final int MidPosition = 80000;
            public static final int HighPosition = 145000;
            public static final int FeederPosition = 138000;
            public static final int standPosition = 21600;
        }

        public static final class Cone {
            public static final int LowPosition = 3000;
            public static final int MidPosition = 136000;
            public static final int HighPosition = 150000;
            public static final int FeederPosition = 135500;
            public static final int standPosition = 49000;
        }

        /* soft limits */
        public static final int forwardSoftLimit = 150000; 
        public static final int reverseSoftLimit = 0;//1000

        /* current limiting */
        public static final int continuousCurrentLimit = 25;
        public static final int peakCurrentLimit = 40;
        public static final double peakCurrentDuration = 0.1;
        public static final boolean enableCurrentLimit = true;

        /*Motion Magic */
        public static final int velocity = 70000;
        public static final int acceleration = 50000;
        public static final int smoothing = 0; // 0-8

        /* PID */
        public static final double elevatorKP = 0.1; 
        public static final double elevatorKI = 0.0;
        public static final double elevatorKD = 0.0;
        public static final double elevatorKF = 0.0;
        //public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
        public static final double closedLoopPeakOutput =1;
    }

    public static final class Wrist {
        public static final int wristID = 40;
        public static final TalonFXConfiguration cofig = new TalonFXConfiguration();
        public static final Boolean wristMotorInvert = true;
        public static final NeutralMode wristNeutralMode = NeutralMode.Brake;

        public static final int Ratio = 15;

        public static final int threshold = 500;

        public static final int SafePosition = 4260;
        public static final int StowPosition = 1200;

        public static final class Cube {
            public static final int LowPosition = 14000;
            public static final int MidPosition = 9700;
            public static final int HighPosition = 12000;
            public static final int FeederPosition = 13000;
            public static final int standPosition = 16500;
        }

        public static final class Cone {
            public static final int LowPosition = 14000;
            public static final int MidPosition = 13242;
            public static final int HighPosition = 9100;
            public static final int FeederPosition = 12000;
            public static final int standPosition = 16500;
        }

        /* soft limits */
        public static final int forwardSoftLimit = 16500; 
        public static final int reverseSoftLimit = 1000;

        /* current limiting */
        public static final int continuousCurrentLimit = 25;
        public static final int peakCurrentLimit = 40;
        public static final double peakCurrentDuration = 0.1;
        public static final boolean enableCurrentLimit = true;

        /* PID */
        public static final double wristKP = 1.5; 
        public static final double wristKI = 0.0;
        public static final double wristKD = 0.0;
        public static final double wristKF = 0.0;
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
        public static final double closedLoopPeakOutput =1; 

        /*Motion Magic */
        public static final int velocity = 7000;
        public static final int acceleration = 7000;
        public static final int smoothing = 0; // 0-8

    }

    public static final class Intake {
        public static final int intakeID = 30; // do not revert : 20
        public static final TalonFXConfiguration cofig = new TalonFXConfiguration();
        public static final Boolean intakeMotorInvert = false;
        public static final NeutralMode intakeNeutralMode = NeutralMode.Brake;

        public static final double speedIn = 0.5;
        public static final double speedOut = 0.3;

        /* current limiting */
        public static final int continuousCurrentLimit = 25;
        public static final int peakCurrentLimit = 40;
        public static final double peakCurrentDuration = 0.1;
        public static final boolean enableCurrentLimit = true;
    }

    public static final class Limelight {

        public static final double limelightHeight = 36.5;
        public static final double nodeHeight = 24;

        public static final class Options{
            public static final int LEDPipe = 0;
            public static final int LEDOff = 1;
            public static final int LEDBlink = 2;
            public static final int LEDOn = 3;
    
            public static final int CAMVision = 0;
            public static final int CAMDrive = 1;
    
            public static final int AprilTag = 0;
            public static final int RetroReflective = 1;
            public static final int RetroReflectiveStand = 2;
            public static final int PIPELINE_3 = 3;
            public static final int PIPELINE_4 = 4;
            public static final int PIPELINE_5 = 5;
            public static final int PIPELINE_6 = 6;
            public static final int PIPELINE_7 = 7;
    
            public static final int STREAM_STANDARD = 0;
            public static final int STREAM_PIP_MAIN = 1;
            public static final int STREAM_PIP_SECONDARY = 2;
    
            public static final int SnapshotOff = 0;
            public static final int SnapshotOn = 1;
        }

        public static final class txAlign {
            public static final TrapezoidProfile.Constraints profile = new TrapezoidProfile.Constraints(0.0001, 0.1); 
            public static final double threshold = 1;
            public static final double kp = 0.08;//.08
            public static final double ki = 0.1;
            public static final double kd = 0.0;
        }

        public static final class tyAlign {
            public static final TrapezoidProfile.Constraints profile = new TrapezoidProfile.Constraints(0.0001, 0.1); 
            public static final double threshold = 1;
            public static final double kp = 0.080;
            public static final double ki = 0.05;
            public static final double kd = 0.0;
        }
        
    }

    public static final class Swerve {
        public static final int pigeonID = 0;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule = 
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(18.5);
        public static final double wheelBase = Units.inchesToMeters(18.5); 
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        public static final double algorithmTickler = 0.0001;

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
        public static final double driveKP = 0.2; 
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.16 / 12); //.32
        public static final double driveKV = (.024 / 12); //1.51
        public static final double driveKA = (0.01 / 12); // .27

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; 
        /** Radians per Second */
        public static final double maxAngularVelocity = 10; 

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */

         /*     Swerve Order
         *         0  1
         *         2  3
         */
        
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 20;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(285.99);
            public static final Rotation2d angleOffsetPractice = Rotation2d.fromDegrees(209.44);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleOffsetPractice);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 21;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(267.89);
            public static final Rotation2d angleOffsetPractice = Rotation2d.fromDegrees(297.24);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset,angleOffsetPractice);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 22;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(100.45);
            public static final Rotation2d angleOffsetPractice = Rotation2d.fromDegrees(289.07);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleOffsetPractice);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 13;
            public static final int canCoderID = 23;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(315.08);
            public static final Rotation2d angleOffsetPractice = Rotation2d.fromDegrees(68.55);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleOffsetPractice);
        }
    }



    public static final class AutoConstants { 
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /*Auto Builder Const */
        public static final PIDConstants translationPID = new PIDConstants(2, 0, 0); //p.5, 1.2
        public static final PIDConstants rotationPID = new PIDConstants(8, 0, 0); //d.05 p15  
    
        /* Constraint for the motion profilied robot angle controller */

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        /*Auto Balance Const */
        public static final double BalanceKp = 0.03; // P (Proportional) constant of a PID loop
        public static final double BalanceGoal = 0;
        public static final double BalanceThreshold = 3;
        public static final double BalanceReverseMulti = .5;
    }
}
