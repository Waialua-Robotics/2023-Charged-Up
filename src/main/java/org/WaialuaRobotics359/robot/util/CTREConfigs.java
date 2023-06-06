package org.WaialuaRobotics359.robot.util;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.autos.swerveBuilderAuto;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.CANcoderConfigurator;
import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.ctre.phoenixpro.configs.MotionMagicConfigs;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.OpenLoopRampsConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenixpro.configs.MagnetSensorConfigs;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

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
                swerveAngleSlot0Config.kF = Constants.Swerve.angleKF;
                swerveAngleFXConfig.Slot0 = swerveAngleSlot0Config;

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
                swerveDriveSlot0Config.kF = Constants.Swerve.driveKF;  
                swerveAngleFXConfig.Slot0 = swerveDriveSlot0Config;      
            
            /* Open Loop Ramp Configuration */
            OpenLoopRampsConfigs swerveDriveOLRConfig = new OpenLoopRampsConfigs();
                swerveDriveOLRConfig.openloopRamp = Constants.Swerve.openLoopRamp;
                swerveDriveOLRConfig.closedloopRamp = Constants.Swerve.closedLoopRamp;//multiple types can be used, ask about it
                swerveAngleFXConfig.OpenLoopRamps = swerveDriveOLRConfig;

        /* Swerve CANCoder Configuration */
            MagnetSensorConfigs swerveCanCoderMSConfig = new MagnetSensorConfigs();
                swerveCanCoderMSConfig.AbsoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
                swerveCanCoderMSConfig.sensorDirection = Constants.Swerve.canCoderInvert;
                swerveCanCoderMSConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
                swerveCanCoderMSConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        /* Slide Motor Configuration */
            /* Current Limit Configuration */
            CurrentLimitsConfigs slideCurrentLimitConfig = new CurrentLimitsConfigs();
                slideCurrentLimitConfig.SupplyCurrentLimitEnable = Constants.Slide.enableCurrentLimit;
                slideCurrentLimitConfig.SupplyCurrentLimit = Constants.Slide.continuousCurrentLimit;
                slideCurrentLimitConfig.SupplyCurrentThreshold = Constants.Slide.peakCurrentLimit;
                slideCurrentLimitConfig.SupplyTimeThreshold = Constants.Slide.peakCurrentDuration;
                slideFXConfig.CurrentLimits = slideCurrentLimitConfig;
            
            /* Soft Limit Configuration */
            HardwareLimitSwitchConfigs slideSoftLimitConfig = new HardwareLimitSwitchConfigs();
                slideSoftLimitConfig.ForwardLimitEnable = true;
                slideSoftLimitConfig.ForwardLimitThreshold = Constants.Slide.forwardSoftLimit;
                slideSoftLimitConfig.ReverseLimitEnable = true;
                slideSoftLimitConfig.ReverseLimitThreshold = Constants.Slide.reverseSoftLimit;
                slideFXConfig.HardwareLimitSwitch = slideSoftLimitConfig;

            /* Slot0 Configuration */
            Slot0Configs slideSlot0Config = new Slot0Configs();
                slideSlot0Config.kP = Constants.Slide.slideKP;
                slideSlot0Config.kI = Constants.Slide.slideKD;
                slideSlot0Config.kD = Constants.Slide.slideKI;
                slideSlot0Config.kF = Constants.Slide.slideKF;        
                slideSlot0Config.closedLoopPeakOutput = Constants.Slide.closedLoopPeakOutput;
                slideFXConfig.Slot0 = slideSlot0Config;

            /* PID Configuration */
            //slideFXConfig.openloopRamp = Constants.Slide.openLoopRamp;
            //slideFXConfig.closedloopRamp = Constants.Slide.closedLoopRamp;

            /* Motion Magic Configuration */
            MotionMagicConfigs slideMotionMagicConfig = new MotionMagicConfigs();
            slideMotionMagicConfig.MotionMagicCruiseVelocity = Constants.Slide.velocity;
            slideMotionMagicConfig.MotionMagicAcceleration = Constants.Slide.acceleration;
            slideMotionMagicConfig.MotionMagicCurveStrength = Constants.Slide.smoothing;
            slideFXConfig.MotionMagic = slideMotionMagicConfig;

        /* Elevator Motor Configuration */
            /* Current Limit Configuration */
            CurrentLimitsConfigs elevatorCurrentLimitConfig = new CurrentLimitsConfigs();
                elevatorCurrentLimitConfig.SupplyCurrentLimitEnable = Constants.Elevator.enableCurrentLimit;
                elevatorCurrentLimitConfig.SupplyCurrentLimit = Constants.Elevator.continuousCurrentLimit;
                elevatorCurrentLimitConfig.SupplyCurrentThreshold = Constants.Elevator.peakCurrentLimit;
                elevatorCurrentLimitConfig.SupplyTimeThreshold = Constants.Elevator.peakCurrentDuration;
                elevatorFXConfig.CurrentLimits = elevatorCurrentLimitConfig;
            
            /* Soft Limit Configuration */
            HardwareLimitSwitchConfigs elevatorSoftLimitConfig = new HardwareLimitSwitchConfigs();
                elevatorSoftLimitConfig.ForwardLimitEnable = true;
                elevatorSoftLimitConfig.ForwardLimitThreshold = Constants.Elevator.forwardSoftLimit;
                elevatorSoftLimitConfig.ReverseLimitEnable = true;
                elevatorSoftLimitConfig.ReverseLimitThreshold = Constants.Elevator.reverseSoftLimit;
                elevatorFXConfig.HardwareLimitSwitch = elevatorSoftLimitConfig;

            /* Slot0 Configuration */
            Slot0Configs elevatorSlot0Config = new Slot0Configs();
                elevatorSlot0Config.kP = Constants.Elevator.elevatorKP;
                //elevatorSlot0Config.kI = Constants.Elevator.elevatorKD;
                //elevatorSlot0Config.kD = Constants.Elevator.elevatorKI;
                //elevatorSlot0Config.kF = Constants.Elevator.elevatorKF;      
                elevatorSlot0Config.closedLoopPeakOutput = Constants.Elevator.closedLoopPeakOutput;
                elevatorFXConfig.Slot0 = elevatorSlot0Config;
            
            /* PID Configuration*/
                //elevatorFXConfig.openloopRamp = Constants.Elevator.openLoopRamp;
                //elevatorFXConfig.closedloopRamp = Constants.Elevator.closedLoopRamp;

            /* Motion Magic Configuration*/
            MotionMagicConfigs elevatorMotionMagicConfig = new MotionMagicConfigs();
                elevatorMotionMagicConfig.MotionMagicCruiseVelocity = Constants.Elevator.velocity;
                elevatorMotionMagicConfig.MotionMagicAcceleration = Constants.Elevator.acceleration;
                elevatorMotionMagicConfig.MotionMagicCurveStrength = Constants.Elevator.smoothing;
                elevatorFXConfig.MotionMagic = elevatorMotionMagicConfig;

        /* Wrist Motor Configuration */
            /* Current Limit Configuration */
            CurrentLimitsConfigs wristCurrentLimitConfig = new CurrentLimitsConfigs();
                wristCurrentLimitConfig.SupplyCurrentLimitEnable = Constants.Wrist.enableCurrentLimit;
                wristCurrentLimitConfig.SupplyCurrentLimit = Constants.Wrist.continuousCurrentLimit;
                wristCurrentLimitConfig.SupplyCurrentThreshold = Constants.Wrist.peakCurrentLimit;
                wristCurrentLimitConfig.SupplyTimeThreshold = Constants.Wrist.peakCurrentDuration;
                wristFXConfig.CurrentLimits = wristCurrentLimitConfig;

            /* Soft limit Configuration */
            HardwareLimitSwitchConfigs wristSoftLimitConfig = new HardwareLimitSwitchConfigs();
                wristSoftLimitConfig.ForwardLimitEnable = true;
                wristSoftLimitConfig.ForwardLimitThreshold = Constants.Wrist.forwardSoftLimit;//not quite sure
                wristSoftLimitConfig.ReverseLimitEnable = true;
                wristSoftLimitConfig.ReverseLimitThreshold = Constants.Wrist.reverseSoftLimit;//same here
                wristFXConfig.HardwareLimitSwitch = wristSoftLimitConfig;
            
            /* Slot0 Configuration */
            Slot0Configs wristSlot0Config = new Slot0Configs();
                wristSlot0Config.kP = Constants.Wrist.wristKP;
                wristSlot0Config.kI = Constants.Wrist.wristKD;
                wristSlot0Config.kD = Constants.Wrist.wristKI;
                wristSlot0Config.kF = Constants.Wrist.wristKF;//replaced with kV (velocity i believe), have to add kS for static friction
                wristSlot0Config.closedLoopPeakOutput = Constants.Wrist.closedLoopPeakOutput; 
                wristFXConfig.Slot0 = wristSlot0Config;

            /* PID Configuration */
            //wristFXConfig.openloopRamp = Constants.Wrist.openLoopRamp;
            //wristFXConfig.closedloopRamp = Constants.Wrist.closedLoopRamp;     

            /*Motion Magic Configuration*/
            MotionMagicConfigs wristMotionMagicConfig = new MotionMagicConfigs();
                wristMotionMagicConfig.MotionMagicCruiseVelocity = Constants.Wrist.velocity;
                wristMotionMagicConfig.MotionMagicAcceleration = Constants.Wrist.acceleration;
                wristMotionMagicConfig.motionCurveStrength = Constants.Wrist.smoothing; //replaced with jerk smoothing
                wristFXConfig.MotionMagic = wristMotionMagicConfig;

            
    }
}