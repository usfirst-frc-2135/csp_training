// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DataLogManager;

public class ExampleSmartMotorController
{
  // Constants
  // TODO: Note that we try to make all numbers use names (these are called literals) to make them describe the value
  //    Use these literals to replace the "magic" numbers in your code--it should improve readability
  private final static int   kSlotIndex              = 0;  // Talon SRX internal slot index for holding PID constnats
  private final static int   kPIDIndex               = 0;  // Talon SRX internal PID index within a slot
  private final static int   kCANTimeout             = 0;  // CTRE timeout that makes the call block and wait for a response
  private final static int   kCTREVelocityConversion = 10; // CTRE reports velocities in counts/100 msec (not seconds)

  // TODO: This constructor is no longer used with the elevator sim, it's causing a crash so I commented it out (stacktrace showed it)
  // public ExampleSmartMotorController(int port)
  // {
  //   // constructor implementation
  // }

  public static final String kEncoderCPR             = null;  // TODO: This probably crashes when run, because it should be an integer, not a string!

  // Also, it is a VARIABLE passed into the constructor, not a constant (m_encoderCPR)
  // enum is short for enumeration, which means the action of mentioning a number of things one by one
  // Often represents specific categories or states, which is why the variables kPosition, kVelocity, 
  //   and kMovementWitchcraft were placed here
  // They prevent errors from arbitrary strings or integers enhances code readability, which makes it 
  //   easier to read
  // Can be used in switch statements

  public enum PIDMode
  {
    kPosition, kVelocity, kMovementWitchcraft,
  }

  WPI_TalonSRX                  m_motor;
  private double                m_kp;
  private double                m_ki;
  private double                m_kd;

  private static double         m_kEncoderCPR;  // TODO: The "k" is not needed
  private double                setpoint;       // TODO: Not used - can delete
  private PIDMode               mode;           // TODO: Not used - can delete
  private TalonSRXSimCollection m_motorSim;

  /**
   * Creates a new ExampleSmartMotorController.
   *
   * @param port
   *          The port for the controller.
   */
  @SuppressWarnings("PMD.UnusedFormalParameter")  // TODO: This suppression is no longer needed, both parameters are now used
  // The constructor initialises the motor with the given PID settings
  // It ensures the motor is ready to operate with these configurations
  public ExampleSmartMotorController(int port, double kEncoderCPR)
  {
    // m_motor = new WPI_TalonSRX(5);  // TODO: This line is not needed, since the port is passed in and only one Talon SRX needs to be created
    //                                  //  TODO: this created a bug where simulation would not work, because the motor sim was on THIS talon SRX and not the one on passed in port
    // This is the constructor; it specifies the CAN bus port that the motor
    // controller is connected to
    m_motor = new WPI_TalonSRX(port); // TODO: I had to move this before the sim collection, you were getting a simulation of the Talon SRX that wasn't being used
    m_motorSim = m_motor.getSimCollection( );
    m_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    setPID(m_kp, m_ki, m_kd);
    DataLogManager.start( );
  }

  public TalonSRXSimCollection getMotorSimulation( )
  {
    return m_motorSim;
  }

  private static double rotationsToCounts(double rotation)
  {
    return rotation * m_kEncoderCPR;
  }

  private static double countsToRotation(double counts)
  {
    return counts / m_kEncoderCPR;
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

    // TODO: It would be nice to log when the PID values change, yes?
    m_motor.config_kP(0, m_kp);
    m_motor.config_kI(0, m_ki);
    m_motor.config_kD(0, m_kd);
    DataLogManager.log("PID");
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
   *          The controller setpoint in rotations or rotations per
   *          second
   * @param arbFeedforward
   *          An arbitrary feedforward output (from -1 to 1 for
   *          percentOutput).
   * 
   */

  //void means you don't return anything
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

      case kMovementWitchcraft :
        controlMode = ControlMode.MotionMagic;
        break;
    }

    m_motor.set(controlMode, rotationsToCounts(setpoint));
    DataLogManager.log("ControlMode " + controlMode + " Setpoint " + setpoint);
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
    return countsToRotation(m_motor.getSelectedSensorPosition( ));
  }

  /**
   * Returns the encoder rate.
   *
   * @return The current encoder rate.
   */
  public double getEncoderRate( )
  {
    return countsToRotation(m_motor.getSelectedSensorVelocity(0) * 10);
  }

  /** Resets the encoder to zero distance. */
  public void resetEncoder( )
  {
    m_motor.setSelectedSensorPosition(0, 0, 0);
    DataLogManager.log("Encoder voltage reset");  // TODO: This is not a voltage--it is an encoder counter
  }

  // TODO: since we're using percentOutput mode, change this parameter name
  // (voltage)
  // There are two different ways to command a motor directly: percentOutput and
  // voltage
  // Mode           percent   Output voltage
  // Full reverse   -1.0      -12.0
  // Stopped        0.0       0.0
  // Full forward   1.0       12.0
  // For this project, we're only using percentOutput modes (for consistency)
  //
  public void set(double percentOutput)
  {
    m_motor.set(ControlMode.PercentOutput, percentOutput);
    DataLogManager.log("percent Voltage: " + ControlMode.PercentOutput);
  }

  public double get( )
  {
    return m_motor.getMotorOutputPercent( );
  }

  public double getClosedLoopError( )
  {
    return (m_motor.getClosedLoopError( ));
  }

  public void setInverted(boolean isInverted)
  {
    m_motor.setInverted(isInverted);
    DataLogManager.log("Motor is inverted"); // TODO: This msg is not really true, the inversion control is set to isInverted (could be true OR false)

  }

  public boolean getInverted( )
  {
    return m_motor.getInverted( );
  }

  public void disable( )
  {
    m_motor.set(ControlMode.Disabled, 0);
    DataLogManager.log("Disabled");
  }

  public void stopMotor( )
  {
    set(0.0);
    DataLogManager.log("Stopped");
  }

}
