// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DataLogManager;

public class ExampleSmartMotorController
{
  // Constants
  // TODO: Note that we try to make all numbers use names (these are called literals) to make them describe the value
  //    Use these literals to replace the "magic" numbers in your code--it should improve readability
  private final static int kSlotIndex              = 0;  // Talon SRX internal slot index for holding PID constnats
  private final static int kPIDIndex               = 0;  // Talon SRX internal PID index within a slot
  private final static int kCANTimeout             = 0;  // CTRE timeout that makes the call block and wait for a response
  private final static int kCTREVelocityConversion = 10; // CTRE reports velocities in counts/100 msec (not seconds)

  // TODO: Maybe it's time to remove these several lines of generic comment about how java works?
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

  // TODO: This second (original) constructor is no longer used with the elevator sim, it's causing a crash so I commented it out (stacktrace showed it)
  // public ExampleSmartMotorController(int port)
  // {
  //   // constructor implementation
  // }

  private int                   m_port;         // CAN ID (port) for the motor controller
  private double                m_encoderCPR;   // Encoder counts per revolution for the attached encoder

  WPI_TalonSRX                  m_motor;        // The Talon SRX motor controller object
  private TalonSRXSimCollection m_motorSim;     // The simulation object for the Talon SRX
  private double                m_kp;
  private double                m_ki;
  private double                m_kd;

  /**
   * Creates a new ExampleSmartMotorController.
   *
   * @param port
   *          The port for the controller.
   * @param encoderCPR
   *          The counts per rotation for the attached encoder.
   */
  @SuppressWarnings("PMD.UnusedFormalParameter")  // TODO: This suppression annotation is no longer needed, both parameters are now used
  // The constructor initialises the motor with the given PID settings
  // It ensures the motor is ready to operate with these configurations
  public ExampleSmartMotorController(int port, double encoderCPR)
  {
    // m_motor = new WPI_TalonSRX(5);  // TODO: This line is not needed, since the port is passed in and only one Talon SRX needs to be created (it's created below)
    //                                  //  TODO: this created a bug where simulation would not work, because the motor sim was on THIS talon SRX and not the one created with the passed in port number
    //
    // This is the constructor; it specifies the CAN bus port that the motor controller is connected to
    m_port = port;
    m_encoderCPR = encoderCPR;

    m_motor = new WPI_TalonSRX(m_port); // TODO: I had to move this line before the sim collection reference, because you were getting a simulation of the Talon SRX that wasn't being created yet
    m_motorSim = m_motor.getSimCollection( ); // TODO: Get the sim object AFTER the motor object is created
    m_motor.configFactoryDefault( );
    m_motor.selectProfileSlot(0, 0);
    m_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    // setPID(m_kp, m_ki, m_kd); // TODO: This does not need to be initialized--make the caller set the PID values!
    // DataLogManager.start( ); // TODO: Only do this ONCE for the entire robot in robotInit - delete this one
  }

  public TalonSRXSimCollection getMotorSimulation( )
  {
    return m_motorSim;
  }

  private double rotationsToCounts(double rotation)
  {
    return rotation * m_encoderCPR;
  }

  private double countsToRotations(double counts)
  {
    return counts / m_encoderCPR;
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
   *          The controller setpoint in rotations or rotations per second
   * @param arbFeedforward
   *          An arbitrary feedforward output (from -1 to 1 for percentOutput).
   */
  //void means you don't return anything  TODO: maybe it's time to remove this generic comment about how java works?
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

    m_motor.set(controlMode, rotationsToCounts(setpoint), DemandType.ArbitraryFeedForward, arbFeedforward);
    DataLogManager.log("Motor controlMode: " + controlMode + " setpoint: " + setpoint);
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
   * @return The current encoder distance in rotations.
   */
  public double getEncoderRotations( )
  {
    return countsToRotations(m_motor.getSelectedSensorPosition( ));
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
    m_motor.setSelectedSensorPosition(0, kPIDIndex, kCANTimeout);
    DataLogManager.log("Motor encoder reset");
  }

  // NOTE:  There are two different ways to command a motor directly: percentOutput and voltage
  //        For this project, we're only using percentOutput modes (for consistency)
  //
  //    Mode           percentOutput voltage
  //    -------------- ------------- -------
  //    Full reverse   -1.0          -12.0
  //    Stopped        0.0           0.0
  //    Full forward   1.0           12.0
  //
  public void setOutput(double percentOutput)
  {
    m_motor.set(ControlMode.PercentOutput, percentOutput);
    DataLogManager.log("Motor percent output: " + percentOutput);
  }

  public double get( )
  {
    return m_motor.getMotorOutputPercent( );
  }

  public double getClosedLoopError( )
  {
    return countsToRotations(m_motor.getClosedLoopError( ));
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
    m_motor.set(ControlMode.Disabled, 0.0);
  }

  public void stopMotor( )
  {
    DataLogManager.log("Motor stopped");
    setOutput(0.0);
  }

}
