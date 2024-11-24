// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ExampleSmartMotorController {
  public static final String kEncoderCPR = null;
  WPI_TalonSRX m_motor;
  private double m_kp;
  private double m_ki;
  private double m_kd;

  // enum is short for enumeration, which means the action of mentioning a number
  // of things one by one
  // Often represents specific categories or states, which is why the variables
  // kPosition, kVelocity, and kMovementWitchcraft were placed here
  // They prevent errors from arbitrary strings or integers
  // enhances code readability, which makes it easier to read
  // Can be used in switch statements
  public enum PIDMode

  {
    kPosition, kVelocity, kMovementWitchcraft,
  }

  

  private static double m_kEncoderCPR;
  private double setpoint;
  private PIDMode mode;
  private TalonSRXSimCollection m_motorSim;

  /**
   * Creates a new ExampleSmartMotorController.
   *
   * @param port
   *             The port for the controller.
   */
  @SuppressWarnings("PMD.UnusedFormalParameter")
  // The constructor initialises the motor with the given PID settings
  // It ensures the motor is ready to operate with these configurations
  public ExampleSmartMotorController(int port) {
    // This is the constructor; it specifies the CAN bus port that the motor
    // controller is connected to
    m_motorSim = m_motor.getSimCollection();
    m_motor = new WPI_TalonSRX(port);
    m_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_motor.config_kP(0, m_kp); // TODO: Now that you're calling setPID a few lines below--these config_xx calls
                                // are redundant, right?
    m_motor.config_kI(0, m_ki);
    m_motor.config_kD(0, m_kd);
    setPID(m_kp, m_ki, m_kd);
  }

  public TalonSRXSimCollection getMotorSimulation() {
    return m_motorSim;
  }

  private static double rotationsToCounts(double rotation) {
    return rotation * m_kEncoderCPR;
  }

  private static double countsToRotation(double counts) {
    return counts / m_kEncoderCPR;
  }

  /**
   * Example method for setting the PID gains of the smart controller.
   *
   * @param kp
   *           The proportional gain.
   * @param ki
   *           The integral gain.
   * @param kd
   *           The derivative gain.
   */
  public void setPID(double kp, double ki, double kd) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;

    // TODO: It would be nice to log when the PID values change, yes?
    m_motor.config_kP(0, m_kp);
    m_motor.config_kI(0, m_ki);
    m_motor.config_kD(0, m_kd);
  }

  public double getKp() {
    return m_kp;
  }

  /**
   * Example method for setting the setpoint of the smart controller in PID mode.
   *
   * @param mode
   *                       The mode of the PID controller.
   * @param setpoint
   *                       The controller setpoint in rotations or rotations per
   *                       second
   * @param arbFeedforward
   *                       An arbitrary feedforward output (from -1 to 1 for
   *                       percentOutput).
   */
  public void setSetpoint(PIDMode mode, double setpoint, double arbFeedforward) {
    ControlMode controlMode;

    switch (mode) {
      default:
      case kPosition:
        controlMode = ControlMode.Position;
        break;

      case kVelocity:
        controlMode = ControlMode.Velocity;
        setpoint /= 10;
        break;

      case kMovementWitchcraft:
        controlMode = ControlMode.MotionMagic;
        break;
    }

    // TODO: It would be nice to log when the setpoint is chaged, yes?
    m_motor.set(controlMode, rotationsToCounts(setpoint));
  }

  /**
   * Places this motor controller in follower mode.
   *
   * @param leader
   *               The leader to follow.
   */
  public void follow(ExampleSmartMotorController leader) {
  }

  /**
   * Returns the encoder distance.
   *
   * @return The current encoder distance.
   */
  public double getEncoderDistance() {
    return countsToRotation(m_motor.getSelectedSensorPosition());
  }

  /**
   * Returns the encoder rate.
   *
   * @return The current encoder rate.
   */
  public double getEncoderRate() {
    return countsToRotation(m_motor.getSelectedSensorVelocity(0) * 10);
  }

  /** Resets the encoder to zero distance. */
  public void resetEncoder() {
    // TODO: It would be nice to log when the encoder is reset, yes?
    m_motor.setSelectedSensorPosition(0, 0, 0);
  }

  // TODO: since we're using percentOutput mode, change this parameter name
  // (voltage)
  // There are two different ways to command a motor directly: percentOutput and
  // voltage
  // Mode percentOutput voltage
  // Full reverse -1.0 -12.0
  // Stopped 0.0 0.0
  // Full forward 1.0 12.0
  // For this project, we're only using percentOutput modes (for consistency)
  //
  public void set(double voltage) {
    // TODO: It would be nice to log the new setting once applied yes?
    m_motor.set(ControlMode.PercentOutput, voltage);
  }

  public double get() {
    return m_motor.getMotorOutputPercent();
  }

  public double getClosedLoopError() {
    return (m_motor.getClosedLoopError());
  }

  public void setInverted(boolean isInverted) {
    // TODO: It would be nice to log when the inversion state is changed, yes?
    m_motor.setInverted(isInverted);
  }

  public boolean getInverted() {
    return m_motor.getInverted();
  }

  public void disable() {
    // TODO: It would be nice to log when the motor is disabled, yes?
    m_motor.set(ControlMode.Disabled, 0);
  }

  public void stopMotor() {
    // TODO: It would be nice to log when the motor gets stopped, yes?
    set(0.0);
  }

}
