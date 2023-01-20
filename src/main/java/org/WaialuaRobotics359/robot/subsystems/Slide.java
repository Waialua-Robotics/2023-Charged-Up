package org.WaialuaRobotics359.robot.subsystems;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Slide extends SubsystemBase{
    private TalonFX mSlideMotor;

    private int desiredPosition;

    public Slide () {
        mSlideMotor = new TalonFX(Constants.Slide.slideMotorID);

        mSlideMotor.configFactoryDefault();
        mSlideMotor.configAllSettings(Robot.ctreConfigs.slideFXConfig);
        mSlideMotor.setInverted(Constants.Slide.slideMotorInvert);
        mSlideMotor.setNeutralMode(Constants.Slide.slideNeutralMode);
    }

    public void setDesiredPosition(int position) {
        desiredPosition = fitToRange(position);
    }

    public int getDesiredPosition() {
        return desiredPosition;
    }

    public void goToPosition() {
        mSlideMotor.set(TalonFXControlMode.Position, desiredPosition);
    }

    public boolean inRange() {
        int encoder = (int) mSlideMotor.getSelectedSensorPosition();
        return (encoder > (desiredPosition - Constants.Slide.threshold)) && (encoder < (desiredPosition + Constants.Slide.threshold));
    }

    private int fitToRange(int position) {
        desiredPosition = Math.min(position, Constants.Slide.forwardSoftLimit);
        desiredPosition = Math.max(desiredPosition, Constants.Slide.reverseSoftLimit);
        return desiredPosition;
    }
}
