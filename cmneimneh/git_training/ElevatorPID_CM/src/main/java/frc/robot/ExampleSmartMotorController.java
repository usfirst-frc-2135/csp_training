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
  // TODO: Note that we try to make all numbers use names (these are called literals) to make them describe the value
  //    Use these literals to replace the "magic" numbers in your code--it should improve readability
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

    //encoder type in TalonSRX is quadrature encoder
    m_motor.selectProfileSlot(0, 0);
    m_motorSim = m_motor.getSimCollection( );
    setPID(m_kp, m_ki, m_kd);
    DataLogManager.start( );
  }

  /**
   * COUNTS TO ROTATIONS
   * encoder reads values in counts, but human users input rotations for simplicity
   * 4096 counts in one rotation, following methods converts counts to rotations, vice versa
   */

  private double rotationsToCounts(double rotation)
  {
    return rotation * m_EncoderCPR;
  }

  private double countsToRotations(double encoderCounts)
  {
    return encoderCounts / m_EncoderCPR;
  }

  /**
   * declares PIDMode as an enum
   * PIDMode used to determine controlmode
   */
  public enum PIDMode
  {
    kPosition, kVelocity, kMovementWitchcraft
  }

  // TODO: This block is no longer used, so delete it. Including the SuppressWarnings
  /**
   * Creates a new ExampleSmartMotorController.
   *
   * @param port
   *          The port for the controller.
   */
  @SuppressWarnings("PMD.UnusedFormalParameter")

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

  /*
   * configure kp, ki, and kd based on values passed in Robot.java
   */
  public void setPID(double kp, double ki, double kd)
  {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
    m_motor.config_kP(0, kp);
    m_motor.config_kI(0, ki);
    m_motor.config_kD(0, kd);
    DataLogManager.log("PID changed");  // TODO: Why not put the kp, ki, and kd right into the log message?
  }

  /*
   * return kp value
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

  //
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
        setpoint /= 10;
        break;

      case kMovementWitchcraft : // is not used in this code
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
    return countsToRotations(m_motor.getSelectedSensorVelocity(0) * 10);
  }

  /** Resets the encoder to zero distance. */
  public void resetEncoder( )
  {
    DataLogManager.log("Encoder reset");
    m_motor.setSelectedSensorPosition(0, 0, 0);
  }

  /**
   * used to set the constant speed of the motor, in percentOutput
   */
  public void set(double percentOutput)
  { // set the speed of the motor using percent output
    m_motor.set(ControlMode.PercentOutput, percentOutput);  // TODO: Why not log a message with the new percent output value?
  }

  public double get( )
  {
    return m_motor.getMotorOutputPercent( );
  }

  /** Inverts motor direction */
  public void setInverted(boolean isInverted)
  {
    DataLogManager.log("Motor inverted"); // TODO: This is where the actual new inverted value should be logged
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
    return countsToRotations(m_motor.getSelectedSensorVelocity( ));
  }

  public TalonSRXSimCollection getMotorSimulation( )
  {
    return m_motorSim;
  }

  public void stopMotor( )
  {
    DataLogManager.log("Motor stopped");
    set(0.0);
  }
}
