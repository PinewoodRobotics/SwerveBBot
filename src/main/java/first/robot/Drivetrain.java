// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveDriveOdometry;
import org.wpilib.math.kinematics.SwerveModulePosition;


/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double MAX_VELOCITY = 3.0; // 3 meters per second
  public static final double MAX_ANGULAR_VELOCITY = Math.PI; // 1/2 rotation per second

  private final Translation2d frontLeftLocation  = new Translation2d(SwerveConstants.kFrontLeftx,  SwerveConstants.kFrontLefty);
  private final Translation2d frontRightLocation = new Translation2d(SwerveConstants.kFrontRightx, SwerveConstants.kFrontRighty);
  private final Translation2d backLeftLocation   = new Translation2d(SwerveConstants.kBackLeftx,   SwerveConstants.kBackLefty);
  private final Translation2d backRightLocation  = new Translation2d(SwerveConstants.kBackRightx,  SwerveConstants.kBackRighty);

  private final SwerveModule frontLeft = new SwerveModule(SwerveConstants.kFrontLeftDriveMotorCanID, SwerveConstants.kFrontLeftTurningMotorCanID, SwerveConstants.kFrontLeftTurningEncoderCanID); //encoder channels still need fixing
  private final SwerveModule frontRight = new SwerveModule(SwerveConstants.kFrontRightDriveMotorCanID, SwerveConstants.kFrontRightTurningMotorCanID, SwerveConstants.kFrontRightTurningEncoderCanID);
  private final SwerveModule backLeft = new SwerveModule(SwerveConstants.kRearLeftDriveMotorCanID, SwerveConstants.kRearLeftTurningMotorCanID, SwerveConstants.kRearLeftTurningEncoderCanID);
  private final SwerveModule backRight = new SwerveModule(SwerveConstants.kRearRightDriveMotorCanID, SwerveConstants.kRearRightTurningMotorCanID, SwerveConstants.kRearRightTurningEncoderCanID);

  private final OnboardIMU imu = new OnboardIMU(OnboardIMU.MountOrientation.FLAT);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          kinematics,
          imu.getRotation2d(),
          new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
          });

/**
 * 
 * 
 * Hi
 * - Andrew C
 * 
 */

  public Drivetrain() {
    imu.resetYaw();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xVelocity Velocity of the robot in the x direction (forward).
   * @param yVelocity Velocity of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y velocities are relative to the field.
   */
  public void drive(
      double xVelocity, double yVelocity, double rot, boolean fieldRelative, double period) {
    var chassisVelocities = new ChassisVelocities(xVelocity, yVelocity, rot);
    if (fieldRelative) {
      chassisVelocities = chassisVelocities.toRobotRelative(imu.getRotation2d());
    }
    chassisVelocities = chassisVelocities.discretize(period);

    var velocities =
        SwerveDriveKinematics.desaturateWheelVelocities(
            kinematics.toWheelVelocities(chassisVelocities), MAX_VELOCITY);

    frontLeft.setDesiredVelocity(velocities[0]);
    frontRight.setDesiredVelocity(velocities[1]);
    backLeft.setDesiredVelocity(velocities[2]);
    backRight.setDesiredVelocity(velocities[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    odometry.update(
        imu.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });
  }
}
