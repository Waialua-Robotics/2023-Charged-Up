package org.WaialuaRobotics359.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;

import com.ctre.phoenixpro.configs.CANcoderConfigurator;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;

import org.WaialuaRobotics359.lib.math.Conversions;
import org.WaialuaRobotics359.lib.util.CTREModuleState;
import org.WaialuaRobotics359.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    public TalonFXConfigurator swerveAngleFXConfigurator = mAngleMotor.getConfigurator();
    public TalonFXConfigurator swerveDriveFXConfigurator = mDriveMotor.getConfigurator();
    public CANcoderConfigurator swerveCANCoderConfigurator = angleEncoder.getConfigurator();

    private DutyCycleOut cyclerequest = new DutyCycleOut(0.0);
    private PositionTorqueCurrentFOC torqueposition = new PositionTorqueCurrentFOC(0.0);
    private PositionVoltage voltageposition = new PositionVoltage(0.0);

    public double CANcoderInitTime = 0.0;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = (!Constants.isCompetitionRobot ? moduleConstants.angleOffsetPractice : moduleConstants.angleOffset);
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, "Drivetrain");
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, "Drivetrain");
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, "Drivetrain");
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public void ForceAngle(int angle){
        mAngleMotor.setControl(torqueposition.withPosition(Conversions.degreesToFalcon(angle, Constants.Swerve.angleGearRatio)));
    }//check if we want torque position or voltage position---------------------------------------------------------------------------------------!!

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(cyclerequest.withOutput(percentOutput));
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        mAngleMotor.setControl(torqueposition.withPosition(Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio)));
        lastAngle = angle;//check if we want torque position vs voltage position----------------------------------------------------------------------!!!!
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getRotorPosition().getValue(), Constants.Swerve.angleGearRatio));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue());
    }

    private void waitForCanCoder(){
        /*
         * Wait for up to 1000 ms for a good CANcoder signal.
         *
         * This prevents a race condition during program startup
         * where we try to synchronize the Falcon encoder to the
         * CANcoder before we have received any position signal
         * from the CANcoder.
         */
        //encodeTimer.start();
        for (int i = 0; i < 100; ++i) {
            angleEncoder.getAbsolutePosition();
            if (angleEncoder.getLastError() == ErrorCode.OK && mAngleMotor.getLastError() == ErrorCode.OK) {
                break;
            }
            Timer.delay(0.010);            
            CANcoderInitTime += 10;
        }
    }

    void resetToAbsolute(){
        waitForCanCoder();
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        mAngleMotor.setRotorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        Timer.delay(.1);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setRotorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(mDriveMotor.getRotorVelocity().getValue(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getRotorPosition().getValue(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        );
    }
}