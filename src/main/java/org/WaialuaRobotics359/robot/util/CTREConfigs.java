package org.WaialuaRobotics359.robot.util;

import org.WaialuaRobotics359.robot.Constants;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.ctre.phoenixpro.configs.FeedbackConfigs;
import com.ctre.phoenixpro.configs.MotionMagicConfigs;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.OpenLoopRampsConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.configs.MagnetSensorConfigs;
import com.ctre.phoenixpro.configs.ClosedLoopRampsConfigs;

public final class CTREConfigs {
    /* Swerve */
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig;
    /* Slide */
    public TalonFXConfiguration slideFXConfig;
    /* Elevator */
    public TalonFXConfiguration elevatorFXConfig;
    /* Wrist */
    public TalonFXConfiguration wristFXConfig;
    /* Intake */
    public TalonFXConfiguration intakeFXConfig;
    /* Fork */
    public TalonFXConfiguration forkFXConfig;


    public CTREConfigs(){
        /* Swerve */
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();
        /* Slide */
        slideFXConfig = new TalonFXConfiguration();
        /* Elevator */
        elevatorFXConfig = new TalonFXConfiguration();
        /* Wrist */
        wristFXConfig = new TalonFXConfiguration();
        /* Intake */
        intakeFXConfig = new TalonFXConfiguration();
        /* Fork */
        forkFXConfig = new TalonFXConfiguration();

        /* Swerve Angle Motor Configurations */
            /* Current Limit Configurations */
            CurrentLimitsConfigs swerveAngleCurrentLimitConfig = new CurrentLimitsConfigs();
                swerveAngleCurrentLimitConfig.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit; 
                swerveAngleCurrentLimitConfig.SupplyCurrentLimit = Constants.Swerve.angleContinuousCurrentLimit;
                swerveAngleCurrentLimitConfig.SupplyCurrentThreshold = Constants.Swerve.anglePeakCurrentLimit;
                swerveAngleCurrentLimitConfig.SupplyTimeThreshold = Constants.Swerve.anglePeakCurrentDuration;
                swerveAngleFXConfig.CurrentLimits = swerveAngleCurrentLimitConfig;

            /* Slot0 Configurations */
            Slot0Configs swerveAngleSlot0Config = new Slot0Configs();
                swerveAngleSlot0Config.kP = Constants.Swerve.angleKP;
                swerveAngleSlot0Config.kI = Constants.Swerve.angleKI;
                swerveAngleSlot0Config.kD = Constants.Swerve.angleKD;
                swerveAngleSlot0Config.kV = Constants.Swerve.angleKF;
                swerveAngleFXConfig.Slot0 = swerveAngleSlot0Config;

            /* Neutral Mode Configuration */
            MotorOutputConfigs swerveAngleMotorOutputConfig = new MotorOutputConfigs();
                swerveAngleMotorOutputConfig.NeutralMode = Constants.Swerve.angleNeutralMode;
                swerveAngleFXConfig.MotorOutput = swerveAngleMotorOutputConfig;

            /* Feedback Configuration */
            FeedbackConfigs swerveAngleMotorFeedbackConfig = new FeedbackConfigs();
                swerveAngleMotorFeedbackConfig.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
                swerveAngleFXConfig.Feedback = swerveAngleMotorFeedbackConfig;

        /* Swerve Drive Motor Configuration */
            /* Current Limit Configuration */
            CurrentLimitsConfigs swerveDriveCurrentLimitConfig = new CurrentLimitsConfigs();
                swerveDriveCurrentLimitConfig.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
                swerveDriveCurrentLimitConfig.SupplyCurrentLimit = Constants.Swerve.driveContinuousCurrentLimit;
                swerveDriveCurrentLimitConfig.SupplyCurrentThreshold = Constants.Swerve.drivePeakCurrentLimit;
                swerveDriveCurrentLimitConfig.SupplyTimeThreshold = Constants.Swerve.drivePeakCurrentDuration;
                swerveDriveFXConfig.CurrentLimits = swerveDriveCurrentLimitConfig;
                
            /* Slot0 Configuration */
            Slot0Configs swerveDriveSlot0Config = new Slot0Configs();
                swerveDriveSlot0Config.kP = Constants.Swerve.driveKP;
                swerveDriveSlot0Config.kI = Constants.Swerve.driveKI;
                swerveDriveSlot0Config.kD = Constants.Swerve.driveKD;
                swerveDriveSlot0Config.kV = Constants.Swerve.driveKF;  
                swerveAngleFXConfig.Slot0 = swerveDriveSlot0Config;      
            
            /* Open Loop Ramp Configuration */
            OpenLoopRampsConfigs swerveDriveOLRConfig = new OpenLoopRampsConfigs();
                swerveDriveOLRConfig.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
                swerveDriveOLRConfig.TorqueOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
                swerveDriveOLRConfig.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
                swerveAngleFXConfig.OpenLoopRamps = swerveDriveOLRConfig;

            ClosedLoopRampsConfigs swerveDriveCLRConfig = new ClosedLoopRampsConfigs();
                swerveDriveCLRConfig.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
                swerveDriveCLRConfig.TorqueClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
                swerveDriveCLRConfig.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
                swerveDriveFXConfig.ClosedLoopRamps = swerveDriveCLRConfig;

            /* Neutral Mode Configuration */
            MotorOutputConfigs swerveDriveMotorOutputConfig = new MotorOutputConfigs();
                swerveDriveMotorOutputConfig.NeutralMode = Constants.Swerve.driveNeutralMode;
                swerveDriveFXConfig.MotorOutput = swerveDriveMotorOutputConfig;

        /* Swerve CANCoder Configuration */
            MagnetSensorConfigs swerveCanCoderMSConfig = new MagnetSensorConfigs();
                swerveCanCoderMSConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
            swerveCanCoderConfig.MagnetSensor = swerveCanCoderMSConfig;

        /* Slide Motor Configuration */
            /* Current Limit Configuration */
            CurrentLimitsConfigs slideCurrentLimitConfig = new CurrentLimitsConfigs();
                slideCurrentLimitConfig.SupplyCurrentLimitEnable = Constants.Slide.enableCurrentLimit;
                slideCurrentLimitConfig.SupplyCurrentLimit = Constants.Slide.continuousCurrentLimit;
                slideCurrentLimitConfig.SupplyCurrentThreshold = Constants.Slide.peakCurrentLimit;
                slideCurrentLimitConfig.SupplyTimeThreshold = Constants.Slide.peakCurrentDuration;
                slideFXConfig.CurrentLimits = slideCurrentLimitConfig;
            
            /* Soft Limit Configuration */
            SoftwareLimitSwitchConfigs slideSoftLimitConfig = new SoftwareLimitSwitchConfigs();
                slideSoftLimitConfig.ForwardSoftLimitEnable = true;
                slideSoftLimitConfig.ForwardSoftLimitThreshold = Constants.Slide.forwardSoftLimit;
                slideSoftLimitConfig.ReverseSoftLimitEnable = true;
                slideSoftLimitConfig.ReverseSoftLimitThreshold = Constants.Slide.reverseSoftLimit;
                slideFXConfig.SoftwareLimitSwitch = slideSoftLimitConfig;

            /* Slot0 Configuration */
            Slot0Configs slideSlot0Config = new Slot0Configs();
                slideSlot0Config.kP = Constants.Slide.slideKP;
                slideSlot0Config.kI = Constants.Slide.slideKD;
                slideSlot0Config.kD = Constants.Slide.slideKI;
                slideSlot0Config.kV = Constants.Slide.slideKF;        
                slideFXConfig.Slot0 = slideSlot0Config;

            /* PID Configuration */
            //slideFXConfig.openloopRamp = Constants.Slide.openLoopRamp;
            //slideFXConfig.closedloopRamp = Constants.Slide.closedLoopRamp;

            /* Motion Magic Configuration */
            MotionMagicConfigs slideMotionMagicConfig = new MotionMagicConfigs();
                slideMotionMagicConfig.MotionMagicCruiseVelocity = Constants.Slide.velocity;
                slideMotionMagicConfig.MotionMagicAcceleration = Constants.Slide.acceleration;
                slideMotionMagicConfig.MotionMagicJerk = Constants.Slide.smoothing;
                slideFXConfig.MotionMagic = slideMotionMagicConfig;

            /* Neutral Mode Configuration */
            MotorOutputConfigs slideMotorOutputConfig = new MotorOutputConfigs();
                slideMotorOutputConfig.NeutralMode = Constants.Slide.slideNeutralMode;
                slideMotorOutputConfig.PeakForwardDutyCycle = Constants.Slide.closedLoopPeakOutput;
                slideMotorOutputConfig.PeakReverseDutyCycle = Constants.Slide.closedLoopReversePeakOutput;
                slideFXConfig.MotorOutput = slideMotorOutputConfig;

        /* Elevator Motor Configuration */
            /* Current Limit Configuration */
            CurrentLimitsConfigs elevatorCurrentLimitConfig = new CurrentLimitsConfigs();
                elevatorCurrentLimitConfig.SupplyCurrentLimitEnable = Constants.Elevator.enableCurrentLimit;
                elevatorCurrentLimitConfig.SupplyCurrentLimit = Constants.Elevator.continuousCurrentLimit;
                elevatorCurrentLimitConfig.SupplyCurrentThreshold = Constants.Elevator.peakCurrentLimit;
                elevatorCurrentLimitConfig.SupplyTimeThreshold = Constants.Elevator.peakCurrentDuration;
                elevatorFXConfig.CurrentLimits = elevatorCurrentLimitConfig;
            
            /* Soft Limit Configuration */
            SoftwareLimitSwitchConfigs elevatorSoftLimitConfig = new SoftwareLimitSwitchConfigs();
                elevatorSoftLimitConfig.ForwardSoftLimitEnable = true;
                elevatorSoftLimitConfig.ForwardSoftLimitThreshold = Constants.Elevator.forwardSoftLimit;
                elevatorSoftLimitConfig.ReverseSoftLimitEnable = true;
                elevatorSoftLimitConfig.ReverseSoftLimitThreshold = Constants.Elevator.reverseSoftLimit;
                elevatorFXConfig.SoftwareLimitSwitch = elevatorSoftLimitConfig;

            /* Slot0 Configuration */
            Slot0Configs elevatorSlot0Config = new Slot0Configs();
                elevatorSlot0Config.kP = Constants.Elevator.elevatorKP;
                //elevatorSlot0Config.kI = Constants.Elevator.elevatorKD;
                //elevatorSlot0Config.kD = Constants.Elevator.elevatorKI;
                //elevatorSlot0Config.kF = Constants.Elevator.elevatorKF;      
                elevatorFXConfig.Slot0 = elevatorSlot0Config;
            
            /* PID Configuration*/
                //elevatorFXConfig.openloopRamp = Constants.Elevator.openLoopRamp;
                //elevatorFXConfig.closedloopRamp = Constants.Elevator.closedLoopRamp;

            /* Motion Magic Configuration*/
            MotionMagicConfigs elevatorMotionMagicConfig = new MotionMagicConfigs();
                elevatorMotionMagicConfig.MotionMagicCruiseVelocity = Constants.Elevator.velocity;
                elevatorMotionMagicConfig.MotionMagicAcceleration = Constants.Elevator.acceleration;
                elevatorMotionMagicConfig.MotionMagicJerk = Constants.Elevator.smoothing;
                elevatorFXConfig.MotionMagic = elevatorMotionMagicConfig;

            /* Neutral Mode Configuration */
            MotorOutputConfigs elevatorMotorOutputConfig = new MotorOutputConfigs();
                elevatorMotorOutputConfig.NeutralMode = Constants.Elevator.elevatorNeutralMode;
                elevatorMotorOutputConfig.PeakForwardDutyCycle = Constants.Elevator.closedLoopPeakOutput;
                elevatorMotorOutputConfig.PeakReverseDutyCycle = Constants.Elevator.closedLoopReversePeakOutput;
                elevatorFXConfig.MotorOutput = elevatorMotorOutputConfig;

        /* Wrist Motor Configuration */
            /* Current Limit Configuration */
            CurrentLimitsConfigs wristCurrentLimitConfig = new CurrentLimitsConfigs();
                wristCurrentLimitConfig.SupplyCurrentLimitEnable = Constants.Wrist.enableCurrentLimit;
                wristCurrentLimitConfig.SupplyCurrentLimit = Constants.Wrist.continuousCurrentLimit;
                wristCurrentLimitConfig.SupplyCurrentThreshold = Constants.Wrist.peakCurrentLimit;
                wristCurrentLimitConfig.SupplyTimeThreshold = Constants.Wrist.peakCurrentDuration;
                wristFXConfig.CurrentLimits = wristCurrentLimitConfig;

            /* Soft limit Configuration */
            SoftwareLimitSwitchConfigs wristSoftLimitConfig = new SoftwareLimitSwitchConfigs();
                wristSoftLimitConfig.ForwardSoftLimitEnable = true;
                wristSoftLimitConfig.ForwardSoftLimitThreshold = Constants.Wrist.forwardSoftLimit;
                wristSoftLimitConfig.ReverseSoftLimitEnable = true;
                wristSoftLimitConfig.ReverseSoftLimitThreshold = Constants.Wrist.reverseSoftLimit;
                wristFXConfig.SoftwareLimitSwitch = wristSoftLimitConfig;
            
            /* Slot0 Configuration */
            Slot0Configs wristSlot0Config = new Slot0Configs();
                wristSlot0Config.kP = Constants.Wrist.wristKP;
                wristSlot0Config.kI = Constants.Wrist.wristKD;
                wristSlot0Config.kD = Constants.Wrist.wristKI;
                wristSlot0Config.kV = Constants.Wrist.wristKF; 
                wristFXConfig.Slot0 = wristSlot0Config;

            /* PID Configuration */
            //wristFXConfig.openloopRamp = Constants.Wrist.openLoopRamp;
            //wristFXConfig.closedloopRamp = Constants.Wrist.closedLoopRamp;     

            /*Motion Magic Configuration*/
            MotionMagicConfigs wristMotionMagicConfig = new MotionMagicConfigs();
                wristMotionMagicConfig.MotionMagicCruiseVelocity = Constants.Wrist.velocity;
                wristMotionMagicConfig.MotionMagicAcceleration = Constants.Wrist.acceleration;
                wristMotionMagicConfig.MotionMagicJerk = Constants.Wrist.smoothing;
                wristFXConfig.MotionMagic = wristMotionMagicConfig;

            /* Neutral Mode Configuration */
            MotorOutputConfigs wristMotorOutputConfig = new MotorOutputConfigs();
                wristMotorOutputConfig.NeutralMode = Constants.Wrist.wristNeutralMode;
                wristMotorOutputConfig.PeakForwardDutyCycle = Constants.Wrist.closedLoopPeakOutput;
                wristMotorOutputConfig.PeakReverseDutyCycle = Constants.Wrist.closedLoopReversePeakOutput;
                wristFXConfig.MotorOutput = wristMotorOutputConfig;


            
    }
}