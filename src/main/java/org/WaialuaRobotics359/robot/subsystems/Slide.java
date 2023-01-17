package org.WaialuaRobotics359.robot.subsystems;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Slide {
    private TalonFX mSlideMotor;

    private int desiredPosition;

    public Slide (int slideID) {
        mSlideMotor = new TalonFX(slideID);

        mSlideMotor.configFactoryDefault();
        mSlideMotor.configAllSettings(Robot.ctreConfigs.slideFXConfig);
        mSlideMotor.setInverted(Constants.Slide.slideMotorInvert);
        mSlideMotor.setNeutralMode(Constants.Slide.slideNeutralMode);
    }

    public void setPosition(int position) {
        desiredPosition = position;
    }

    public int getPosition() {
        return desiredPosition;
    }

    public boolean inRange() {
        int encoder = (int) mSlideMotor.getSelectedSensorPosition();
        return (encoder > (desiredPosition - Constants.Slide.threshold)) && (encoder < (desiredPosition + Constants.Slide.threshold));
    }

    public void periodic() {
        mSlideMotor.set(TalonFXControlMode.Position, desiredPosition);
    }
    
}
