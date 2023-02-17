package org.WaialuaRobotics359.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.WaialuaRobotics359.robot.Constants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;


public class LEDs extends SubsystemBase {
    private final CANdle candle;

    private final StrobeAnimation PurpleStrobe;
    private final StrobeAnimation YellowStrobe;

    public static Boolean hasObject = false; 

    public static enum State {off,purple,yellow,Blue,Red}
    public State state = State.off;

    public LEDs() {
        candle = new CANdle(Constants.LEDs.CANdleID, "rio");
        PurpleStrobe = new StrobeAnimation(147, 0, 255, 0, .2, Constants.LEDs.LEDCount, 0);
        YellowStrobe = new StrobeAnimation(247, 255, 0, 0, .2, Constants.LEDs.LEDCount, 0);

        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 1; // dim the LEDs to half brightness

        candle.configAllSettings(config);
        //setLEDAliance ();
    }

    @Override
    public void periodic(){
        switch(state){
            case off:
                candle.setLEDs(0,0,0);
                break;
            case purple:
                if (hasObject){
                    candle.animate(PurpleStrobe, 1);
                }else{
                    candle.setLEDs(147,0,255);
                    candle.clearAnimation(1);
                }
                break;
            case yellow:
                if (hasObject){
                    candle.animate(YellowStrobe, 1);
                }else{
                    candle.setLEDs(247,255,0);
                    candle.clearAnimation(1);
                }
                break;
            case Blue:
                candle.setLEDs(0,0,255);
                break;
            case Red:
                candle.setLEDs(255,0,0);
                break;
            default:
                candle.setLEDs(0,0,0);
                break;
        }
    }

    public void setLEDs(int i, int j, int k) {
    }

    public void setLEDAliance (){
        candle.clearAnimation(1);
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue){
            state = State.Blue;
        } else {
            state = State.Red;
        }
    }

}
   