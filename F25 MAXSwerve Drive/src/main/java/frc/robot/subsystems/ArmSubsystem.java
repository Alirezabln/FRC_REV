// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController; 
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig; 
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType; 
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.AbsoluteEncoder;

/**
 * Subsystem for controlling a two-motor jointed arm using position PID control 
 * with gravity compensation (Arbitrary Feed-Forward) and motion profiling limits.
 * This version uses the explicit SparkMaxConfig object, MAXMotion, and the 
 * recommended nested configuration style (closedLoop) for REVLib 2025.
 * * NOTE: Position tracking now relies solely on the lead motor's (armupper) encoder
 * and uses the **position conversion factor** so the PID operates directly in **degrees**.
 */
public class ArmSubsystem extends SubsystemBase {
  // Constants
  private static final double GEAR_RATIO = 27.418; // Encoder Rotations per 1 Arm Revolution
  private static final double ARM_HORIZONTAL_OFFSET = 100.5; // Angle (in degrees) where the arm is horizontal
  private static final int CURRENT_LIMIT_AMPS = 80;
  // NOTE: MAX_VELOCITY and MAX_ACCELERATION are now in Degrees/sec and Degrees/sec^2
  private static final double MAX_VELOCITY = 10000.0; // Adjusted for degree units (10 Rotations/sec * 360/rot)
  private static final double MAX_ACCELERATION= 20000.0; // Adjusted for degree units (20 Rotations/sec^2 * 360/rot)
  public double calcFF;
  
  // Motor Controllers (Upper is the lead motor)
  public SparkMax armupper = new SparkMax(26, MotorType.kBrushless);
  public SparkMax armlower = new SparkMax(25, MotorType.kBrushless);

  // Encoders and PID Controller
  // Only using the encoder from the lead motor (armupper) for feedback.
  RelativeEncoder  armupEncoder = armupper.getEncoder();
  
  // Updated class name: SparkPIDController is now SparkClosedLoopController
  private final SparkClosedLoopController m_pidController;

  // PID Tuning Variables (kept as instance variables for periodic tuning)
  private double kP, kI, kD, kIz, kFF_Scalar, kMaxOutput, kMinOutput, ERR_ALLOW;
  
  private double m_setPoint_Degrees; // Target position in degrees

  public ArmSubsystem() {
    // 1. Initial PID Coefficients and Limits
    kP = 0.01; 
    kI = 0;
    kD = 0.001; 
    kIz = 0; 
    kFF_Scalar = 0.97;
    kMaxOutput = 1.0; // Max output percentage (voltage limit)
    kMinOutput = -1.0; // Min output percentage (voltage limit)
    ERR_ALLOW = 0.50; // degree error allowed

    // 2. Instantiate PID Controller
    m_pidController = armupper.getClosedLoopController();

    // 3. Structured Motor Configuration using SparkMaxConfig and MAXMotion (2025 Style)
    
    // Create a configuration object
    SparkMaxConfig armConfig = new SparkMaxConfig();
    SparkMaxConfig lowerConfig = new SparkMaxConfig();

    lowerConfig.follow(armupper);
    
    // --- General Motor Parameters ---
    armConfig.idleMode(IdleMode.kBrake); // Arm systems should typically use Brake mode
    armConfig.smartCurrentLimit(CURRENT_LIMIT_AMPS); 
    
    // --- Encoder Conversion Factor ---
    // Sets the encoder to output Degrees of arm rotation instead of raw motor rotations
    // initializes position at zero
    armConfig.encoder.positionConversionFactor(360.0 / GEAR_RATIO); 
    armupEncoder.setPosition(0);

    
    // --- Closed Loop and MAXMotion Configuration (2025 Chained Style) ---
    // Configure all PID, Output Range, and Motion Limits in Slot 0
    armConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // Explicitly set the feedback sensor (Best Practice)
        .p(kP)
        .i(kI)
        .d(kD)
        .velocityFF(0.0) // Internal kFF is 0.0 (using Arbitrary FF)
        .iZone(kIz)
        .outputRange(kMinOutput, kMaxOutput);
        
    armConfig.closedLoop.maxMotion // Nested accessor for MAXMotion parameters
          // Max Velocity and Acceleration are now in Deg/s and Deg/s^2 
          .maxVelocity(MAX_VELOCITY) 
          .maxAcceleration(MAX_ACCELERATION)
          .allowedClosedLoopError(ERR_ALLOW);
    
    armupper.configure(armConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    armlower.configure(lowerConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // 6. Display Tuning Variables on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("FF Gravity Scalar", kFF_Scalar);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Angle (Deg)", 0);
  }

  /**
   * Calculates the current arm position in degrees relative to vertical (0 degrees).
   */
  public double armPos_Degrees() {
    // The encoder is now configured to report position directly in degrees.
    return armupEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // 1. Read and update PID coefficients from SmartDashboard
    // This allows real-time tuning without needing to create and re-apply the config object.

   
    // 2. Calculate Gravity Compensation (Arbitrary Feed-Forward)
    double currentAngle = -armPos_Degrees();
    calcFF = kFF_Scalar * Math.sin(Math.toRadians(currentAngle - ARM_HORIZONTAL_OFFSET));

    // 3. Apply position reference and arbitrary Feed-Forward
 

    // 4. Display Telemetry
    SmartDashboard.putNumber("Arm Position (Deg)", currentAngle);
    SmartDashboard.putNumber("Target Degrees", m_setPoint_Degrees);
    SmartDashboard.putNumber("Calculated FF", calcFF);
    SmartDashboard.putNumber("Arm Output", armupper.getAppliedOutput());
    
  }

  /**
   * Sets the desired target angle for the arm.
   * @param angleInDegrees The desired angle of the arm in degrees.
   */
  public void setAngle(double angleInDegrees) {
    // The setpoint is now directly in degrees due to the conversion factor applied to the encoder.
    System.out.println("settings Pos in Subsystem");
    m_pidController.setReference(
        -angleInDegrees, // Setpoint is now in Degrees
        ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0,// Gravity compensation applied here
        calcFF
    );

    // Put the target angle on the dashboard for confirmation
    SmartDashboard.putNumber("Set Angle (Deg)", angleInDegrees);
  }
}
