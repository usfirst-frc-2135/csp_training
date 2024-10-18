// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * A simplified stub class that simulates the API of a common "smart" motor controller.
 *
 * <p>Has no actual functionality.
 */
public class ExampleSmartMotorController {

  private final WPI_TalonSRX m_motor = new WPI_TalonSRX(5);
  private double m_kp; 
  private double m_ki; 
  private double m_kd; 

  public enum PIDMode {
    kPosition,
    kVelocity,
    kMovementWitchcraft
  }

  /**
   * Creates a new ExampleSmartMotorController.
   *
   * @param port The port for the controller.
   */
  @SuppressWarnings("PMD.UnusedFormalParameter")
  public ExampleSmartMotorController(int port) {}

  /**
   * Example method for setting the PID gains of the smart controller.
   *
   * @param kp The proportional gain.
   * @param ki The integral gain.
   * @param kd The derivative gain.
   */
  public void setPID(double kp, double ki, double kd) {
    m_kp = kp; 
    m_ki = ki; 
    m_kd = kd; 
  }

  /**
   * Example method for setting the setpoint of the smart controller in PID mode.
   *
   * @param mode The mode of the PID controller.
   * @param setpoint The controller setpoint.
   * @param arbFeedforward An arbitrary feedforward output (from -1 to 1).
   */
  public void setSetpoint(PIDMode mode, double setpoint, double arbFeedforward) {}

  /**
   * Places this motor controller in follower mode.
   *
   * @param leader The leader to follow.
   */
  public void follow(ExampleSmartMotorController leader) {}

  /**
   * Returns the encoder distance.
   *
   * @return The current encoder distance.
   */
  public double getEncoderDistance() {
    return 0;
  }

  /**
   * Returns the encoder rate.
   *
   * @return The current encoder rate.
   */
  public double getEncoderRate() {
    return 0;
  }

  /** Resets the encoder to zero distance. */
  public void resetEncoder() {
    m_motor.setSelectedSensorPosition(0);
  }

  public void set(double speed) {}

  public double get() {
    return 0;
  }

  public void setInverted(boolean isInverted) {}

  public boolean getInverted() {
    return false;
  }

  public void disable() {}

  public void stopMotor() {
    m_motor.set(ControlMode.PercentOutput, 0);
  }
}
