package org.WaialuaRobotics359.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.WaialuaRobotics359.robot.Constants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;


public class LEDs extends SubsystemBase {
    private final CANdle candle;

    public static enum State {off,purple,yellow}
    public State state = State.off;

    public LEDs() {
        candle = new CANdle(Constants.LEDs.CANdleID, "rio");

        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 1; // dim the LEDs to half brightness

        candle.configAllSettings(config);
    }

    @Override
    public void periodic(){
        switch(state){
            case off:
                candle.setLEDs(0,0,0);
                break;
            case purple:
                candle.setLEDs(147,0,255);
                break;
            case yellow:
                candle.setLEDs(247,255,0);
                break;
            default:
                candle.setLEDs(0,0,0);
                break;
        }
    }

  
}
   