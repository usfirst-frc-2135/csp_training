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
 * Wrapper of motor controller
 */
public class ExampleSmartMotorController
{
  /**
   * declares PIDMode as an enum used to determine controlmode
   */
  public enum PIDMode
  {
    kPosition, kVelocity, kMovementWitchcraft
  }

  private final static int      kSlotIndex              = 0;  // Talon SRX internal slot index for holding PID constnats
  private final static int      kPIDIndex               = 0;  // Talon SRX internal PID index within a slot
  private final static int      kCANTimeout             = 0;  // CTRE timeout that makes the call block and wait for a response
  private final static int      kCTREVelocityConversion = 10; // CTRE reports velocities in counts/100 msec (not seconds)

  private WPI_TalonSRX          m_motor;
  private TalonSRXSimCollection m_motorSim;

  private double                m_kp;
  private double                m_ki;
  private double                m_kd;

  private double                m_encoderCPR;

  /**
   * Creates a new ExampleSmartMotorController.
   *
   * @param port
   *          The port for the controller.
   */
  public ExampleSmartMotorController(int ports, double encoderCPR)
  {
    m_motor = new WPI_TalonSRX(ports);
    m_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_motor.selectProfileSlot(kSlotIndex, kPIDIndex);

    setPID(m_kp, m_ki, m_kd);

    m_motorSim = m_motor.getSimCollection( );
    m_encoderCPR = encoderCPR;
  }

  /**
   * COUNTS TO ROTATIONS
   * encoder reads values in counts, but human users input rotations for simplicity
   * 4096 counts in one rotation, following methods converts counts to rotations, vice versa
   */
  private double rotationsToCounts(double rotations)
  {
    return rotations * m_encoderCPR;
  }

  /**
   * ROTATIONS TO COUNTS
   * encoder reads values in counts, but human users input rotations for simplicity
   * 4096 counts in one rotation, following methods converts counts to rotations, vice versa
   */
  private double countsToRotations(double encoderCounts)
  {
    return encoderCounts / m_encoderCPR;
  }

  /**
   * Example method for setting the PID gains of the smart controller.
   * configure kp, ki, and kd based on values passed in Robot.java
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
    DataLogManager.log("PID Changed:  kP " + kp + " kI " + ki + " kd " + kd);
    m_motor.config_kP(kSlotIndex, kp);
    m_motor.config_kI(kSlotIndex, ki);
    m_motor.config_kD(kSlotIndex, kd);
  }

  /**
   * Return Kp value to Robot.java
   */
  public double getKp( )
  {
    return m_kp;
  }

  /**
   * Example method for setting the setpoint of the smart controller in PID mode.
   * Define the control mode based on the desired type of PID: positional, veolcity, or motion magic
   * Desired setpoint, in rotations, is converted to counts
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

      case kMovementWitchcraft : // is not used in this code
        controlMode = ControlMode.MotionMagic;
        break;
    }

    // DataLogManager.log("ControlMode: " + controlMode + " Setpoint: " + setpoint);
    m_motor.set(controlMode, rotationsToCounts(setpoint));
  }

  /**
   * Places this motor controller in follower mode.
   * Method is not used in Robot.java
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
    return countsToRotations(m_motor.getSelectedSensorPosition(kPIDIndex));
  }

  /**
   * Returns the encoder rate.
   *
   * @return The current encoder rate.
   */
  public double getEncoderRate( )
  {
    return countsToRotations(m_motor.getSelectedSensorVelocity(0) * kCTREVelocityConversion);
  }

  /** Resets the encoder to zero distance. */
  public void resetEncoder( )
  {
    DataLogManager.log("Encoder Reset :)");
    m_motor.setSelectedSensorPosition(0, kSlotIndex, kCANTimeout);
  }

  /**
   * used to set the constant speed of the motor, in percentOutput
   * Change the variable name and comment.
   */
  public void set(double percentOutput)
  {
    DataLogManager.log("Encoder Percent Output Changed");
    m_motor.set(ControlMode.PercentOutput, percentOutput);
  }

  public double get( )
  {
    return m_motor.getMotorOutputPercent( );
  }

  /** Inverts motor direction */
  public void setInverted(boolean isInverted)
  {
    DataLogManager.log("Motor Inverted: " + ControlMode.PercentOutput);
    m_motor.setInverted(isInverted);
  }

  public boolean getInverted( )
  {
    return m_motor.getInverted( );
  }

  public double getClosedLoopError( )
  {
    return (m_motor.getClosedLoopError( ));
  }

  public void disable( )
  {
    DataLogManager.log("Motor disabled");
    m_motor.set(ControlMode.Disabled, 0);
  }

  public double getVelocity( )
  {
    return m_motor.getSelectedSensorVelocity( );
  }

  public void stopMotor( )
  {
    DataLogManager.log("Motor stopped");
    set(0.0);
  }

  /**
   * Return the motor simulation object for this controller
   *
   * @return motor simulation object
   */
  public TalonSRXSimCollection getMotorSimulation( )
  {
    return m_motorSim;
  }
}
