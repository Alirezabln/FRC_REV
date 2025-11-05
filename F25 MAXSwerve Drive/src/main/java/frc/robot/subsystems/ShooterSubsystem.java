package frc.robot.subsystems;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkFlex m_leftMotor; //These might be top and bottom
  private final SparkFlex m_rightMotor;
  private final SparkClosedLoopController m_leftController;
  private final SparkClosedLoopController m_rightController;
  //private static final int currentLimit = 80;
  //config.currentLimit = 40; // Example current limit

  //Motor constants
  private static final int LEFT_CAN_ID = 22; //I don't remember these lol
  private static final int RIGHT_CAN_ID = 21;
  private static final double RPM_TARGET = 2000.0; // example RPM
  private static final double TOLERANCE_RPM = 100.0; // example tolerance


public ShooterSubsystem() {
    // Initialize motor
    m_leftMotor = new SparkFlex(LEFT_CAN_ID, MotorType.kBrushless);
    m_rightMotor = new SparkFlex(RIGHT_CAN_ID, MotorType.kBrushless);
    m_leftController = m_leftMotor.getClosedLoopController();
    m_rightController = m_rightMotor.getClosedLoopController();

    
    SparkFlexConfig config = new SparkFlexConfig();
    // configure encoder (Idk if this is neeeded)
    config.encoder
        .positionConversionFactor(1.0) 
        .velocityConversionFactor(1.0);



    // configure PID
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.0001, com.revrobotics.spark.ClosedLoopSlot.kSlot0)
        .i(0.0, com.revrobotics.spark.ClosedLoopSlot.kSlot0)
        .d(0.0, com.revrobotics.spark.ClosedLoopSlot.kSlot0)
        .outputRange(-1.0, 1.0, com.revrobotics.spark.ClosedLoopSlot.kSlot0); //What does this do?

    // feed‑forward for velocity (kF is calculated as 1023 / RPM at full output)
    config.closedLoop
        .velocityFF(1.0 / 5767, com.revrobotics.spark.ClosedLoopSlot.kSlot0);


    // configure MAXMotion (motion profiling)
    config.closedLoop.maxMotion
        .cruiseVelocity(double RPM_TARGET, ClosedLoopSlot kSlot0)
        .maxAcceleration(5000, com.revrobotics.spark.ClosedLoopSlot.kSlot0)
        .allowedProfileError( TOLERANCE_RPM, com.revrobotics.spark.ClosedLoopSlot.kSlot0 );

    // Apply config to both motors
    m_leftMotor.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
    m_rightMotor.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);

    // Make right motor follow left motor or invert (check the configuration of the motors I forgor :skull:)
    m_rightMotor.follow(m_leftMotor, true); 

    SmartDashboard.putNumber("Shooter Target RPM", RPM_TARGET);
    SmartDashboard.putNumber("Shooter Actual RPM", 0.0);
    SmartDashboard.putBoolean("Shooter At Speed", false);
  }

  @Override
  public void periodic() {
    double actualRPM = m_leftMotor.getEncoder().getVelocity();
    SmartDashboard.putNumber("Shooter Actual RPM", actualRPM);
    boolean atSpeed = Math.abs(actualRPM - SmartDashboard.getNumber("Shooter Target RPM", RPM_TARGET)) < TOLERANCE_RPM;
    SmartDashboard.putBoolean("Shooter At Speed", atSpeed);
  }

  /** Spin the shooter motors to the target RPM. */
  public void spinUp(double rpm) {
    SmartDashboard.putNumber("Shooter Target RPM", rpm);
    m_leftController.setSetpoint(rpm, com.revrobotics.spark.SparkBase.ControlType.kVelocity, com.revrobotics.spark.ClosedLoopSlot.kSlot0);
  }

  /** Stop the shooter motors. */
  public void stop() {
    m_leftMotor.set(0.0);
    // m_rightMotor follows automatically
  }

  /** Returns true when the shooter is at the target RPM within tolerance. */
  public boolean isAtSpeed() {
    double currentRPM = m_leftMotor.getEncoder().getVelocity();
    double targetRPM = SmartDashboard.getNumber("Shooter Target RPM", RPM_TARGET);
    return Math.abs(currentRPM - targetRPM) < TOLERANCE_RPM;
  }
}

  