package org.WaialuaRobotics359.robot.subsystems;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;

import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.MotionMagicVoltage;
import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Slide extends SubsystemBase{
    private TalonFX mSlideMotor;
    private double desiredPosition = 0;

    /* Motion Magic & Percent Output */
    private MotionMagicVoltage MotionMagic = new MotionMagicVoltage(1);
    private DutyCycleOut cyclerequest = new DutyCycleOut(1);
    public TalonFXConfigurator slideFXConfigurator;

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

        slideFXConfigurator = mSlideMotor.getConfigurator();
        slideFXConfigurator.apply(Robot.ctreConfigs.slideFXConfig);
        mSlideMotor.setInverted(Constants.Slide.slideMotorInvert);
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
        desiredPosition /= 2048;
    }

    public double getDesiredPosition() {
        return desiredPosition;
    }

    public void currentToDisired(){
        setDesiredPosition((int)GetPosition());
    }


    public void goToPosition() {
        mSlideMotor.setControl(MotionMagic.withPosition(desiredPosition).withSlot(0));
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
        mSlideMotor.setControl(cyclerequest.withOutput(percent));
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
            mSlideMotor.setControl(cyclerequest.withOutput(-.1));
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

    private double fitToRange(double position) {
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
