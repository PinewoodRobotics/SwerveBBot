// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import org.wpilib.hardware.bus.CANPort;
import com.revrobotics.RelativeEncoder;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.trajectory.TrapezoidProfile;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SwerveModule {
  private static final double WHEEL_RADIUS = 0.0508; //need to fix these later
  private static final int ENCODER_RESOLUTION = 4096;

  private static final double MODULE_MAX_ANGULAR_VELOCITY = Drivetrain.MAX_ANGULAR_VELOCITY;
  private static final double MODULE_MAX_ANGULAR_ACCELERATION =
      2 * Math.PI; // radians per second squared

  private final SparkMax driveMotor;
  private final SparkMax turningMotor;

  private final SparkMaxConfig driveConfig ;  
  private final SparkMaxConfig turningConfig;



  private final CANcoder turningEncoder;

  private final RelativeEncoder driveEncoder;
  //private final Encoder turningEncoder;

  

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController drivePIDController = new PIDController(0.1, 0, 0); //need to fix these later

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController turningPIDController = //need to fix these later
      new ProfiledPIDController(
          1.5,
          0.0015,
          0.12,
          new TrapezoidProfile.Constraints(
              MODULE_MAX_ANGULAR_VELOCITY, MODULE_MAX_ANGULAR_ACCELERATION));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorCanId CAN ID for the drive motor.
   * @param turningMotorCanId CAN ID for the turning motor.
   */
  public SwerveModule(
      int driveMotorCanId,
      int turningMotorCanId,
      int turningEncoderCanId
      ) {
      driveMotor = new SparkMax(CANPort.CAN_S0, driveMotorCanId, MotorType.kBrushless);
      turningMotor = new SparkMax(CANPort.CAN_S0, turningMotorCanId, MotorType.kBrushless); // need to fix S0
      turningEncoder = new CANcoder(turningEncoderCanId, new CANBus(CANPort.CAN_S0));  

      driveConfig = new SparkMaxConfig();
      driveConfig.idleMode(IdleMode.kBrake);
      driveConfig.smartCurrentLimit(40);
      driveConfig.inverted(false); 

      driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
      turningConfig = new SparkMaxConfig();
      turningConfig.idleMode(IdleMode.kBrake);
      turningConfig.smartCurrentLimit(20);
      turningConfig.inverted(false);

      turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      driveEncoder = driveMotor.getEncoder();

    //turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
   
    //driveEncoder.setDistancePerPulse(2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION);

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    
    //turningEncoder.setDistancePerPulse(2 * Math.PI / ENCODER_RESOLUTION);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }
  private double getTurningAngleRadians() {
    return turningEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
  }
  private double getDriveDistanceMeters() {
    return driveEncoder.getPosition().get() / DRIVE_GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS;
  }
  private double getDriveVelocityMetersPerSec() {
    return driveEncoder.getVelocity().get() / 60.0 / DRIVE_GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS;
  }

  private static final double DRIVE_GEAR_RATIO = 6.75; // need to fix/update later

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */

  
  public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistanceMeters(), new Rotation2d(getTurningAngleRadians()));
    
  }

  /**
   * Returns the current velocity of the module.
   *
   * @return The current velocity of the module.
   */
  
  public SwerveModuleVelocity getVelocity() {
    return new SwerveModuleVelocity(
        getDriveVelocityMetersPerSec(), new Rotation2d(getTurningAngleRadians()));
  }

  /**
   * Sets the desired velocity for the module.
   *
   * @param desiredVelocity Desired velocity.
   */
  public void setDesiredVelocity(SwerveModuleVelocity desiredVelocity) {
    var encoderRotation = new Rotation2d(getTurningAngleRadians());

    // Optimize the desired velocity to avoid spinning further than 90 degrees, then scale velocity
    // by cosine of angle error. This scales down movement perpendicular to the desired direction of
    // travel that can occur when modules change directions. This results in smoother driving.
    SwerveModuleVelocity velocity =
        desiredVelocity.optimize(encoderRotation).cosineScale(encoderRotation);

    // Calculate the drive output from the drive PID controller and feedforward.

    final double driveOutput =
        drivePIDController.calculate(getDriveVelocityMetersPerSec(), velocity.velocity)
            + driveFeedforward.calculate(desiredVelocity.velocity);

    // Calculate the turning motor output from the turning PID controller and feedforward.
    final double turnOutput =
        turningPIDController.calculate(getTurningAngleRadians(), velocity.angle.getRadians())
            + turnFeedforward.calculate(turningPIDController.getSetpoint().velocity);

    driveMotor.setVoltage(driveOutput);
    turningMotor.setVoltage(turnOutput);
  }
}
