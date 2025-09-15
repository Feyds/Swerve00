package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.util.function.DoubleSupplier;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class SwerveSubsystem extends SubsystemBase {

 //   public static SwerveSubsystem instance;

  public static final double MAX_SPEED = 4.5; // meters per second

  public static final double MAX_ANGULAR_VELOCITY = Math.PI * 2; // radians per second

  private final SwerveDrive swerveDrive;

  private final Field2d field2d;

  /* 
  public static SwerveSubsystem getInstance() {
    if(instance == null) {
        instance = new SwerveSubsystem(File directory);
    }
    return instance;
  } */

  public SwerveSubsystem(File directory) {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(MAX_SPEED);
    } catch (Exception e) {
      throw new RuntimeException("Failed to create swerve drive", e);
    }

    field2d = new Field2d();
    SmartDashboard.putData("Field", field2d);

    swerveDrive.setHeadingCorrection(false);
  }

  public void drive(DoubleSupplier translationX, DoubleSupplier translationY, 
                   DoubleSupplier angularRotationX, boolean fieldRelative) {

    double xVelocity = translationX.getAsDouble() * MAX_SPEED;
    double yVelocity = translationY.getAsDouble() * MAX_SPEED;
    double angularVelocity = angularRotationX.getAsDouble() * MAX_ANGULAR_VELOCITY;

    swerveDrive.drive(
        new Translation2d(xVelocity, yVelocity),
        angularVelocity,
        fieldRelative,
        false 
    );
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    swerveDrive.drive(chassisSpeeds);
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void postTrajectory(Trajectory trajectory) {
    if (trajectory != null) {
      field2d.getObject("Trajectory").setTrajectory(trajectory);
    }
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public void lock() {
    swerveDrive.lockPose();
  }

  public Rotation2d getHeading() {
    return swerveDrive.getYaw();
  }

   /* 
  public ChassisSpeeds getChassisSpeeds() {
    return swerveDrive.getChassisVelocity;
  } */

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();

    field2d.setRobotPose(getPose());

    SmartDashboard.putNumber("Robot Pose X", getPose().getX());
    SmartDashboard.putNumber("Robot Pose Y", getPose().getY());
    SmartDashboard.putNumber("Robot Heading", getHeading().getDegrees());
    

    /* 
    // Put chassis speeds on dashboard
    ChassisSpeeds speeds = getChassisSpeeds();
    SmartDashboard.putNumber("Chassis Vx", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Chassis Vy", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Chassis Omega", speeds.omegaRadiansPerSecond);
    */
  }

  @Override
  public void simulationPeriodic() {
    // Simulation-specific updates can go here
  }
}