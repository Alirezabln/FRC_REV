package frc.robot.subsystems;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class ShooterSubsystem extends SubsystemBase {
  private SparkFlex motor;
  private SparkFlexConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;

public ShooterSubsystem() {
    // Initialize motor
    motor = new SparkFlex(30, MotorType.kBrushless);

    // Create and apply motor configuration
    motorConfig = new SparkFlexConfig();
    motorConfig.idleMode = SparkFlexConfig.IdleMode.kCoast;
    motorConfig.currentLimit = 40; // Example current limit
    motor.apply(motorConfig, ResetMode.kResetToFactoryDefaults);

    // Get encoder
    encoder = motor.getEncoder();

    // Configure closed-loop control
    closedLoopController = motor.getClosedLoopController();
    ClosedLoopSlot slot = closedLoopController.getSlot(0);
    slot.setFeedbackSensor(FeedbackSensor.kIntegratedSensor);
    slot.setP(0.1);
    slot.setI(0.0);
    slot.setD(0.0);
    slot.setFF(0.0);
    closedLoopController.setOutputRange(-1.0, 1.0);
    motor.persist(PersistMode.kPersistAll);
  }

  