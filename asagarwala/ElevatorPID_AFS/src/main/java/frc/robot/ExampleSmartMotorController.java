// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DataLogManager;

/**
 * A simplified stub class that simulates the API of a common "smart" motor
 * controller.
 *
 * <p>
 * Has no actual functionality.
 */

public class ExampleSmartMotorController
{
  // Constants
  // TODO: Note that we try to make all numbers use names (these are called literals) to make them describe the value
  //    Use these literals to replace the "magic" numbers in your code--it should improve readability
  private final static int kSlotIndex              = 0;  // Talon SRX internal slot index for holding PID constnats
  private final static int kPIDIndex               = 0;  // Talon SRX internal PID index within a slot
  private final static int kCANTimeout             = 0;  // CTRE timeout that makes the call block and wait for a response
  private final static int kCTREVelocityConversion = 10; // CTRE reports velocities in counts/100 msec (not seconds)

  public enum PIDMode
  {
    kPosition, kVelocity, kMovementWitchcraft
  }

  private double                m_kp = 0.8;           // (native units) 10% * 102.3 / 1023
  private double                m_ki = 0.0;
  private double                m_kd = 0.0;

  private int                   m_port;
  private WPI_TalonSRX          m_motor;
  private TalonSRXSimCollection m_motorSim;
  private double                m_encoderCPR;

  // TODO: Don't know what this constructor was for, but it's not needed
  // public ExampleSmartMotorController(double kP, double kV, double kMV)
  // {
  // }

  /**
   * Creates a new ExampleSmartMotorController.
   * 
   * @param port
   *          The port for the controller.
   */
  public ExampleSmartMotorController(int port, double encoderCPR)
  {
    m_port = port;
    m_encoderCPR = encoderCPR;

    m_motor = new WPI_TalonSRX(m_port);
    m_motorSim = m_motor.getSimCollection( );

    m_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_motor.selectProfileSlot(kSlotIndex, kCANTimeout);
    setPID(m_kp, m_ki, m_kd);
  }

  public TalonSRXSimCollection getMotorSimulation( )
  {
    return m_motorSim;
  }

  private double countsToRotations(double encoderCounts)
  {
    return encoderCounts / m_encoderCPR;
  }

  private double rotationsToCounts(double rotation)
  {
    return rotation * m_encoderCPR;
  }

  /**
   * Example method for setting the PID gains of the smart controller.
   *
   * @param kp
   *          The proportional gain.
   * @param ki
   *          The integral gain.
   * @param kd
   *          The derivative gain.
   */
  public void setPID(double kp, double ki, double kd)
  {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
    m_motor.config_kP(kSlotIndex, m_kp);
    m_motor.config_kI(kSlotIndex, m_ki);
    m_motor.config_kD(kSlotIndex, m_kd);
    DataLogManager.log("Motor PID settings: kp " + m_kp + " ki " + m_ki + " kd " + kd);
  }

  public double getkP( )
  {
    return m_kp;
  }

  /**
   * Example method for setting the setpoint of the smart controller in PID mode.
   *
   * @param mode
   *          The mode of the PID controller.
   * @param setpoint
   *          The controller setpoint.
   * @param arbFeedforward
   *          An arbitrary feedforward output (from -1 to 1).
   */
  public void setSetpoint(PIDMode mode, double setpoint, double arbFeedforward)
  {
    ControlMode controlMode;
    switch (mode)
    {
      default :
      case kPosition :
        controlMode = ControlMode.Position;
        break;

      case kVelocity :
        controlMode = ControlMode.Velocity;
        setpoint /= kCTREVelocityConversion;
        break;

      case kMovementWitchcraft :
        controlMode = ControlMode.MotionMagic;
        break;
    }

    DataLogManager.log("Motor controlMode: " + controlMode + " setpoint: " + setpoint);
    m_motor.set(controlMode, rotationsToCounts(setpoint));
  }

  /**
   * Places this motor controller in follower mode.
   *
   * @param leader
   *          The leader to follow.
   */
  public void follow(ExampleSmartMotorController leader)
  {}

  /**
   * Returns the encoder distance in rotations.
   *
   * @return The current encoder distance.
   */
  public double getEncoderRotations( )
  {
    return countsToRotations(m_motor.getSelectedSensorPosition( ));
  }

  public double getClosedLoopError(){
    return countsToRotations(m_motor.getClosedLoopError());
  }

  /**
   * Returns the encoder rate.
   *
   * @return The current encoder rate.
   */
  public double getEncoderRate( )
  {
    return countsToRotations(m_motor.getSelectedSensorVelocity(kPIDIndex) * kCTREVelocityConversion);
  }

  /** Resets the encoder to zero distance. */
  public void resetEncoder( )
  {
    DataLogManager.log("Motor encoder reset");
    m_motor.setSelectedSensorPosition(kSlotIndex, kPIDIndex, 0);
  }

  public void set(double percentOutput)
  {
    DataLogManager.log("Motor percent output: " + percentOutput);
    m_motor.set(ControlMode.PercentOutput, percentOutput);
  }

  public double get( )
  {
    return m_motor.getMotorOutputPercent( );
  }

  public void setInverted(boolean isInverted)
  {
    DataLogManager.log("Motor: " + ((isInverted) ? "" : "NOT ") + "Inverted");
    m_motor.setInverted(isInverted);
  }

  public boolean getInverted( )
  {
    return m_motor.getInverted( );
  }

  public void disable( )
  {
    DataLogManager.log("Motor disabled");
    m_motor.set(ControlMode.Disabled, 0);
  }

  public void stopMotor( )
  {
    DataLogManager.log("Motor stopped");
    set(0.0);
  }
}
