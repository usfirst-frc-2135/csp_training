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
 * A simplified stub class that simulates the API of a common "smart" motor controller.
 *
 * <p>
 * Has no actual functionality.
 */

public class ExampleSmartMotorController
{
  // Constants
  private final static int      kSlotIndex              = 0;  // Talon SRX internal slot index for holding PID constnats
  private final static int      kPIDIndex               = 0;  // Talon SRX internal PID index within a slot
  private final static int      kCANTimeout             = 0;  // CTRE timeout that makes the call block and wait for a response
  private final static int      kCTREVelocityConversion = 10; // CTRE reports velocities in counts/100 msec (not seconds)

  // Class member objects
  private WPI_TalonSRX          m_motor;
  private double                m_kp;
  private double                m_ki;
  private double                m_kd;
  private double                m_EncoderCPR;

  private TalonSRXSimCollection m_motorSim;

  public ExampleSmartMotorController(int ports, double kEncoderCPR)
  {
    m_motor = new WPI_TalonSRX(ports);
    m_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    m_EncoderCPR = kEncoderCPR;

    //Encoder type in TalonSRX is quadrature encoder
    m_motor.selectProfileSlot(kSlotIndex, kPIDIndex);
    m_motorSim = m_motor.getSimCollection( );
    setPID(m_kp, m_ki, m_kd);
    DataLogManager.start( );
  }

  /**
   * Converts the encoder value from rotations to counts.
   *
   * @param encoderRotations The value in rotations.
   * @return The encoder value converted from rotations to counts.
   */

  private double rotationsToCounts(double rotation)
  {
    return rotation * m_EncoderCPR;
  }

  /**
   * Converts the encoder value from counts to rotations.
   *
   * @param encoderCounts The value in counts.
   * @return The encoder value converted from rotations to counts.
   */

  private double countsToRotations(double encoderCounts)
  {
    return encoderCounts / m_EncoderCPR;
  }

  /**
   * Declares PIDMode as an enum
   * PIDMode used to determine controlmode
   */

  public enum PIDMode
  {
    kPosition, kVelocity, kMovementWitchcraft
  }

  /**
   * Example method for setting the PID gains of the smart controller.
   *
   * @param kp The proportional gain
   * @param ki The integral gain
   * @param kd The derivative gain
   */

  public void setPID(double kp, double ki, double kd)
  {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
    m_motor.config_kP(kSlotIndex, kp);
    m_motor.config_kI(kSlotIndex, ki);
    m_motor.config_kD(kSlotIndex, kd);
    DataLogManager.log("PID changed. kp: " + kp + ", ki: " + ki + ", kd: " + kd);
  }

  /*
   * Return kp value
   */

  public double getKp( )
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

      case kMovementWitchcraft : // not used in this code
        controlMode = ControlMode.MotionMagic;
        break;
    }

    m_motor.set(controlMode, rotationsToCounts(setpoint));
    DataLogManager.log("ControlMode: " + controlMode + " Setpoint: " + setpoint);
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
   * Returns the encoder distance.
   *
   * @return The current encoder distance.
   */

  public double getEncoderDistance( )
  {
    return countsToRotations(m_motor.getSelectedSensorPosition(0));
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
    DataLogManager.log("Encoder reset");
    m_motor.setSelectedSensorPosition(0, kPIDIndex, kCANTimeout);
  }

  /**
   * Used to set the constant speed of the motor, in percentOutput
   */

  public void set(double percentOutput)
  {
    m_motor.set(ControlMode.PercentOutput, percentOutput);
    DataLogManager.log("Percent output: " + percentOutput);
  }

  /**
   * Returns the constant speed of the motor in percentOutput
   * @return MotorOutputPercent
   */

  public double get( )
  {
    return m_motor.getMotorOutputPercent( );
  }

  /** Inverts motor direction */
  public void setInverted(boolean isInverted)
  {
    DataLogManager.log("Motor inverted: " + ControlMode.PercentOutput);
    m_motor.setInverted(isInverted);
  }

  /**
   * Return to get whether motor is inverted or not
   * @return True if motor is inverted, false if it is not
   */

  public boolean getInverted( )
  {
    return m_motor.getInverted( );
  }

  /**
   * Returns difference between desired target value and motor's actual value
   * @return ClosedLoopError
   */

  public double getClosedLoopError( )
  {
    return (m_motor.getClosedLoopError( ));
  }

  /**
   * Disables motor
   */

  public void disable( )
  {
    DataLogManager.log("Motor disabled");
    m_motor.set(ControlMode.Disabled, 0);
  }

  /**
   * Returns velocity as measured by encoder in rotations
   * @return Velocity in rotations
   */

  public double getVelocity( )
  {
    return countsToRotations(m_motor.getSelectedSensorVelocity( ));
  }

  public TalonSRXSimCollection getMotorSimulation( )
  {
    return m_motorSim;
  }

  /**
   * Stops motor by setting percentOutput to 0
   */

  public void stopMotor( )
  {
    DataLogManager.log("Motor stopped");
    set(0.0);
  }
}
