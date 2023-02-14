package org.WaialuaRobotics359.robot.commands.swerve;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.LimeLight;
import org.WaialuaRobotics359.robot.subsystems.Swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PoseEstimator extends CommandBase {

    private final Swerve s_Swerve;
    private final LimeLight s_LimeLight;

     // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.
  private static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
  private static final Matrix<N1, N1> localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.01));
  private static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();

  public PoseEstimator(LimeLight s_LimeLight, Swerve s_Swerve) {
    this.s_LimeLight = s_LimeLight;
    this.s_Swerve = s_Swerve;

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    poseEstimator = new SwerveDrivePoseEstimator(
        s_Swerve.getYaw(),
        new Pose2d(),
        Constants.Swerve.swerveKinematics, stateStdDevs,
        localMeasurementStdDevs, visionMeasurementStdDevs);
    
    
    tab.addString("Pose (X, Y)", this::getFomattedPose).withPosition(0, 4);
    tab.addNumber("Pose Degrees", () -> getCurrentPose().getRotation().getDegrees()).withPosition(1, 4);
    tab.add(field2d);
  }
    
}
