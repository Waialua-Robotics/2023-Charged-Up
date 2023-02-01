package org.WaialuaRobotics359.robot.util;

import java.util.Map;

import org.WaialuaRobotics359.robot.RobotContainer;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Dashboard {
    public Dashboard(RobotContainer container) {
        ShuffleboardTab tab = Shuffleboard.getTab("Subsystem");

        tab.addNumber("elevator Height", () -> container.getElevator().GetPositionInches()).withSize(2, 2)
                .withPosition(0, 0).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min" , 0 , "max" , 49));
        tab.addNumber("slide Height", () -> container.getSlide().GetPositionInches()).withSize(2, 2)
                .withPosition(3, 0).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min" , 0 , "max" , 25));
        tab.addNumber("Wrist Angle", () -> container.getWrist().GetPositionInches()).withSize(2, 2)
                .withPosition(6, 0).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min" , 0 , "max" , 25));
        tab.addNumber("Wrist value", () -> container.getWrist().GetPosition());
        tab.addNumber("Slide value", () -> container.getSlide().GetPosition());
        tab.addNumber("Elevator value", () -> container.getElevator().GetPosition());
        tab.addNumber("Wrist desired", () -> container.getWrist().getDesiredPosition());
        tab.addNumber("Slide desired", () -> container.getSlide().getDesiredPosition());
        tab.addNumber("Elevator desired", () -> container.getElevator().getDesiredPosition());
        tab.addBoolean("isCube", () -> RobotContainer.isCube);
        tab.addBoolean("DriveSlowMode", () -> RobotContainer.DriveSlowMode);

        /*tab.add("Autonomous Mode", container.getAutonomousChooser().getModeChooser()).withSize(2, 1).withPosition(2, 0);
        tab.add("Climb Mode", container.getClimbChooser().getClimbChooser()).withSize(2, 1).withPosition(0, 2);
        tab.addCamera("Camera", "Camera", "http://limelight.local:5800", "http://10.29.10.11:5800").withSize(3, 3)
                .withPosition(4, 0);*/
    }
    
}
