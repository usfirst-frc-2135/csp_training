// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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

  private final static double kEncoderCPR = 4096;

  private double rotationsToCounts(double rotation) {
    return rotation * kEncoderCPR;
  }

  private double countsToRotations(double encoderCounts) {
    return encoderCounts / kEncoderCPR;
  }

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
  public ExampleSmartMotorController(int port) {
    m_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_motor.selectProfileSlot(0, 0);

    m_motor.config_kP(0, m_kp);
    m_motor.config_kI(0, m_ki);
    m_motor.config_kD(0, m_kd);
  }

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
  public void setSetpoint(PIDMode mode, double setpoint, double arbFeedforward) {
    ControlMode controlMode;

    switch (mode) {
      default :
      case kPosition :
        controlMode = ControlMode.Position;
        break;

      case kVelocity :
        controlMode = ControlMode.Velocity;
        setpoint /= 10;
        break;

      case kMovementWitchcraft : // is not used in this code
        controlMode = ControlMode.MotionMagic;
        break;
    }

    m_motor.set(controlMode, rotationsToCounts(setpoint));
  }

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
    return countsToRotations(m_motor.getSelectedSensorPosition(0));
  }

  /**
   * Returns the encoder rate.
   *
   * @return The current encoder rate.
   */
  public double getEncoderRate() {
    return countsToRotations(m_motor.getSelectedSensorVelocity(0) * 10);
  }

  /** Resets the encoder to zero distance. */
  public void resetEncoder() {
    m_motor.setSelectedSensorPosition(0, 0, 0);
  }

  public void set(double voltage) {
    m_motor.set(ControlMode.PercentOutput, voltage);
  }

  public double get() {
    return m_motor.getMotorOutputPercent();
  }

  public void setInverted(boolean isInverted) {
    m_motor.setInverted(isInverted);
  }

  public boolean getInverted() {
    return m_motor.getInverted( );
  }

  public void disable() {
    m_motor.set(ControlMode.Disabled, 0);
  }

  public void stopMotor() {
    set(0.0);
  }
}
