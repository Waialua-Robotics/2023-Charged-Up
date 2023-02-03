package org.WaialuaRobotics359.robot.util;

import org.WaialuaRobotics359.robot.Constants;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    /* Swerve */
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;
    /* Slide */
    public TalonFXConfiguration slideFXConfig;
    /* Elevator */
    public TalonFXConfiguration elevatorFXConfig;
    /* Wrist */
    public TalonFXConfiguration wristFXConfig;
    /* Intake */
    public TalonFXConfiguration intakeFXConfig;

    public CTREConfigs(){
        /* Swerve */
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();
        /* Slide */
        slideFXConfig = new TalonFXConfiguration();
        /* Elevator */
        elevatorFXConfig = new TalonFXConfiguration();
        /* Wrist */
        wristFXConfig = new TalonFXConfiguration();
        /* Intake */
        intakeFXConfig = new TalonFXConfiguration();

        /* Swerve Angle Motor Configurations */
            SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
                Constants.Swerve.angleEnableCurrentLimit, 
                Constants.Swerve.angleContinuousCurrentLimit, 
                Constants.Swerve.anglePeakCurrentLimit, 
                Constants.Swerve.anglePeakCurrentDuration);

            swerveAngleFXConfig.slot0.kP = Constants.Swerve.angleKP;
            swerveAngleFXConfig.slot0.kI = Constants.Swerve.angleKI;
            swerveAngleFXConfig.slot0.kD = Constants.Swerve.angleKD;
            swerveAngleFXConfig.slot0.kF = Constants.Swerve.angleKF;
            swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        /* Swerve Drive Motor Configuration */
            SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
                Constants.Swerve.driveEnableCurrentLimit, 
                Constants.Swerve.driveContinuousCurrentLimit, 
                Constants.Swerve.drivePeakCurrentLimit, 
                Constants.Swerve.drivePeakCurrentDuration);

            swerveDriveFXConfig.slot0.kP = Constants.Swerve.driveKP;
            swerveDriveFXConfig.slot0.kI = Constants.Swerve.driveKI;
            swerveDriveFXConfig.slot0.kD = Constants.Swerve.driveKD;
            swerveDriveFXConfig.slot0.kF = Constants.Swerve.driveKF;        
            swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
            swerveDriveFXConfig.openloopRamp = Constants.Swerve.openLoopRamp;
            swerveDriveFXConfig.closedloopRamp = Constants.Swerve.closedLoopRamp;
        /* Swerve CANCoder Configuration */
            swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
            swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
            swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
            swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        /* Slide Motor Configuration */
            SupplyCurrentLimitConfiguration slideSupplyLimit = new SupplyCurrentLimitConfiguration(
                Constants.Slide.enableCurrentLimit, 
                Constants.Slide.continuousCurrentLimit, 
                Constants.Slide.peakCurrentLimit, 
                Constants.Slide.peakCurrentDuration);

            slideFXConfig.forwardSoftLimitEnable = true;
            slideFXConfig.forwardSoftLimitThreshold = Constants.Slide.forwardSoftLimit;
            slideFXConfig.reverseSoftLimitEnable = true;
            slideFXConfig.reverseSoftLimitThreshold = Constants.Slide.reverseSoftLimit;

            slideFXConfig.slot0.kP = Constants.Slide.slideKP;
            slideFXConfig.slot0.kI = Constants.Slide.slideKD;
            slideFXConfig.slot0.kD = Constants.Slide.slideKI;
            slideFXConfig.slot0.kF = Constants.Slide.slideKF;        
            slideFXConfig.supplyCurrLimit = slideSupplyLimit;
            slideFXConfig.slot0.closedLoopPeakOutput = Constants.Slide.closedLoopPeakOutput;
            /*PID */
            //slideFXConfig.openloopRamp = Constants.Slide.openLoopRamp;
            //slideFXConfig.closedloopRamp = Constants.Slide.closedLoopRamp;

            /*Magic*/
            slideFXConfig.motionCruiseVelocity = Constants.Slide.velocity;
            slideFXConfig.motionAcceleration = Constants.Slide.acceleration;
            slideFXConfig.motionCurveStrength = Constants.Slide.smoothing;

        /* Elevator Motor Configuration */
            SupplyCurrentLimitConfiguration elevatorSupplyLimit = new SupplyCurrentLimitConfiguration(
                Constants.Elevator.enableCurrentLimit, 
                Constants.Elevator.continuousCurrentLimit, 
                Constants.Elevator.peakCurrentLimit, 
                Constants.Elevator.peakCurrentDuration);

            elevatorFXConfig.forwardSoftLimitEnable = true;
            elevatorFXConfig.forwardSoftLimitThreshold = Constants.Elevator.forwardSoftLimit;
            elevatorFXConfig.reverseSoftLimitEnable = true;
            elevatorFXConfig.reverseSoftLimitThreshold = Constants.Elevator.reverseSoftLimit;

            elevatorFXConfig.slot0.kP = Constants.Elevator.elevatorKP;
            elevatorFXConfig.slot0.kI = Constants.Elevator.elevatorKD;
            elevatorFXConfig.slot0.kD = Constants.Elevator.elevatorKI;
            elevatorFXConfig.slot0.kF = Constants.Elevator.elevatorKF;      
            elevatorFXConfig.supplyCurrLimit = elevatorSupplyLimit;
            elevatorFXConfig.slot0.closedLoopPeakOutput = Constants.Elevator.closedLoopPeakOutput;
            
            /*PID*/
            //elevatorFXConfig.openloopRamp = Constants.Elevator.openLoopRamp;
            //elevatorFXConfig.closedloopRamp = Constants.Elevator.closedLoopRamp;

            /*Magic*/
            elevatorFXConfig.motionCruiseVelocity = Constants.Slide.velocity;
            elevatorFXConfig.motionAcceleration = Constants.Slide.acceleration;
            elevatorFXConfig.motionCurveStrength = Constants.Slide.smoothing;
        /* Wrist Motor Configuration */
            SupplyCurrentLimitConfiguration wristSupplyLimit = new SupplyCurrentLimitConfiguration(
                Constants.Wrist.enableCurrentLimit, 
                Constants.Wrist.continuousCurrentLimit, 
                Constants.Wrist.peakCurrentLimit, 
                Constants.Wrist.peakCurrentDuration);

            wristFXConfig.forwardSoftLimitEnable = true;
            wristFXConfig.forwardSoftLimitThreshold = Constants.Wrist.forwardSoftLimit;
            wristFXConfig.reverseSoftLimitEnable = true;
            wristFXConfig.reverseSoftLimitThreshold = Constants.Wrist.reverseSoftLimit;

            wristFXConfig.slot0.kP = Constants.Wrist.wristKP;
            wristFXConfig.slot0.kI = Constants.Wrist.wristKD;
            wristFXConfig.slot0.kD = Constants.Wrist.wristKI;
            wristFXConfig.slot0.kF = Constants.Wrist.wristKF;        
            wristFXConfig.supplyCurrLimit = wristSupplyLimit;
            wristFXConfig.slot0.closedLoopPeakOutput = Constants.Wrist.closedLoopPeakOutput; 
            /*PID */
            //wristFXConfig.openloopRamp = Constants.Wrist.openLoopRamp;
            //wristFXConfig.closedloopRamp = Constants.Wrist.closedLoopRamp;     

            /*Magic*/
            wristFXConfig.motionCruiseVelocity = Constants.Wrist.velocity;
            wristFXConfig.motionAcceleration = Constants.Wrist.acceleration;
            wristFXConfig.motionCurveStrength = Constants.Wrist.smoothing;
    }
}