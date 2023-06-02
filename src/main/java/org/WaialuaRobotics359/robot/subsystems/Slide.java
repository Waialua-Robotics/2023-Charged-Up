package org.WaialuaRobotics359.robot.subsystems;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.ControlModeValue;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Slide extends SubsystemBase{
    private TalonFX mSlideMotor;
    private int desiredPosition = 0;

    /*Logging*/
    private DataLog logger;
    /*elevator logs*/
    private DoubleLogEntry slideDesiredPosition;
    private DoubleLogEntry slideCurrentPosition;
    private DoubleLogEntry slideMotorCurrent;
    private DoubleLogEntry slideMotorVelocity;
    private DoubleLogEntry slideMotorTemperature;

    public Slide () {
        mSlideMotor = new TalonFX(Constants.Slide.slideMotorID);

        mSlideMotor.getConfigurator().apply(new TalonFXConfiguration());
        mSlideMotor.configAllSettings(Robot.ctreConfigs.slideFXConfig);
        mSlideMotor.setInverted(Constants.Slide.slideMotorInvert);
        mSlideMotor.setNeutralMode(Constants.Slide.slideNeutralMode);
        mSlideMotor.setRotorPosition(0);

        /*Logging*/
        logger = DataLogManager.getLog();
        slideDesiredPosition = new DoubleLogEntry(logger, "slide/desiredPosition");
        slideCurrentPosition = new DoubleLogEntry(logger, "slide/currentPosition");
        slideMotorCurrent = new DoubleLogEntry(logger, "slide/motorCurrent");
        slideMotorVelocity = new DoubleLogEntry(logger, "slide/motorVelocity");
        slideMotorTemperature = new DoubleLogEntry(logger, "slide/motorTemperature");
    }

    public void setDesiredPosition(double position) {
        desiredPosition = fitToRange(position);
    }

    public int getDesiredPosition() {
        return desiredPosition;
    }

    public void currentToDisired(){
        setDesiredPosition(GetPosition());
    }


    public void goToPosition() {
        mSlideMotor.set(TalonFXControlMode.MotionMagic, desiredPosition);
    }

    public boolean inRange() {
        double encoder = mSlideMotor.getPosition().getValue();
        return (encoder > (desiredPosition - Constants.Slide.threshold)) && (encoder < (desiredPosition + Constants.Slide.threshold));
    }

    public double GetPosition() {
        return mSlideMotor.getPosition().getValue();
    }

    public double GetPositionInches(){
        return  (GetPosition()*(Constants.Slide.Ratio))/61440;
    }

    public void SetPrecentOut(double percent){
        mSlideMotorset(TalonFXControlMode.PercentOutput, percent);
    }

    public void Stop(){
        SetPrecentOut(0);
    }

    public void AutoZero() {
        final double ZeroCurrent = .1;
        if (mSlideMotor.getStatorCurrent().getValue() > ZeroCurrent){
            SetHomePosition();
            return;
        }else{
        mSlideMotor.set(TalonFXControlMode.PercentOutput, -.1);
        }   
    } 

    public double getCurrent(){
        return mSlideMotor.getStatorCurrent().getValue();
    }

    public void SetPosition(double position){
        mSlideMotor.setRotorPosition(position);
    }

    public void SetHomePosition(){
        mSlideMotor.setRotorPosition(0);
    }

    private int fitToRange(int position) {
        desiredPosition = Math.min(position, Constants.Slide.forwardSoftLimit);
        desiredPosition = Math.max(desiredPosition, Constants.Slide.reverseSoftLimit);
        return desiredPosition;
    }

    private void LogData(long time){
        slideDesiredPosition.append(desiredPosition, time);
        slideCurrentPosition.append(GetPosition(), time);
        slideMotorCurrent.append(mSlideMotor.getStatorCurrent().getValue(), time);
        slideMotorVelocity.append(mSlideMotor.getVelocity().getValue(), time);
        slideMotorTemperature.append(mSlideMotor.getDeviceTemp().getValue(), time);
    }

    public void periodic(){
        LogData(RobotController.getFPGATime());
    }
}
