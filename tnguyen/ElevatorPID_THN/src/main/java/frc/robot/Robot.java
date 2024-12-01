// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot
{
  // Constants
  // TODO: Note that we try to make all numbers use names (these are called literals) to make them describe the value
  private final static double               kDt              = 0.02;  // Loop delay time for simulation
  private final static int                  kGamepadPort     = 0;     // XBox controller USB port
  private final static int                  kMotorCANId      = 5;     // Motor CAN ID assignment
  private final static double               kP               = 0.38;  // PID - proportional value
  private final static double               kI               = 0.0;   // PID - integral value
  private final static double               kD               = 0.0;   // PID - derivative value
  private final static double               kEncoderCPR      = 4096;  // Encoder CPR for CTRE Mag encoder connected to Talon SRX
  private final static double               kConstantOutput  = 0.3;   // Constant percent output value
  private final static double               kMaxVelocity     = 1.0;   // Trapezoidal profile max velocity
  private final static double               kMaxAcceleration = 1.0;   // Trapezoidal profile max acceleration
  private final static double               kForwardGoal     = 3.0;   // Trapezoidal move - forward goal rotations
  private final static double               kReverseGoal     = 0.0;   // Trapezoidal move - reverse goal rotations

  //  private final ExampleSmartMotorController m_ExampleSmartMotorController = new ExampleSmartMotorController(); // TODO: Delete this old stuff, yes?

  // Class member objects
  private final XboxController              m_controller     = new XboxController(kGamepadPort);
  private final ExampleSmartMotorController m_motor          = new ExampleSmartMotorController(kMotorCANId, kEncoderCPR);
  // private final SimpleMotorFeedforward      m_feedforward    = new SimpleMotorFeedforward(1, 1.5); // TODO: This isn't used or needed
  private final TrapezoidProfile            m_profile        =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration));
  private TrapezoidProfile.State            m_goal           = new TrapezoidProfile.State( );
  private TrapezoidProfile.State            m_setpoint       = new TrapezoidProfile.State( );

  private final TalonSRXSimCollection       m_motorSim       = m_motor.getMotorSimulation( );
  private final ElevSim                     m_elevSim        = new ElevSim(m_motorSim, kEncoderCPR);

  private double                            goal;
  private boolean                           m_pidEnabled     = false;

  @Override
  public void robotInit( )
  {
    DataLogManager.start( );  // TODO: Placing this as the first call, will ensure the following methods will actually get logged, right?
    // Note: These gains are fake, and will have to be tuned for your robot. // TODO: Now that these are tuned, do we need this comment? (It's now quite wrong)
    m_motor.setPID(0.38, 0.0, 0.0);
    m_motor.resetEncoder( );
    m_elevSim.periodic( );
  }

  @Override
  public void teleopPeriodic( )
  {
    // TODO: Any reason to keep all of this stuff? Seems like you've grown way past needing it.
    /*
     * if (m_joystick.getRawButtonPressed(2)) {
     * m_goal = new TrapezoidProfile.State(5, 0);
     * } else if (m_joystick.getRawButtonPressed(3)) {
     * m_goal = new TrapezoidProfile.State();
     * }
     * 
     * // Retrieve the profiled setpoint for the next timestep. This setpoint moves
     * // toward the goal while obeying the constraints.
     * m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
     * 
     * // Send setpoint to offboard controller PID
     * m_motor.setSetpoint(
     * ExampleSmartMotorController.PIDMode.kPosition,
     * m_setpoint.position,
     * m_feedforward.calculate(m_setpoint.velocity) / 12.0);
     */

    // SmartDashboard.putNumber("Elevator Rotations", m_motor.getEncoderDistance());
    // SmartDashboard.putNumber("Target", goal);
    // SmartDashboard.putNumber("Error", m_motor.getClosedLoopError());

    SmartDashboard.putNumber("Elevator Rotations", m_motor.getEncoderDistance( ));
    SmartDashboard.putNumber("Target", goal);
    SmartDashboard.putNumber("Error", m_motor.getClosedLoopError( ));
    SmartDashboard.putNumber("Kp", m_motor.getKp( ));   // TODO: Since this cannot be changed while running, it probably should be in robotInit (not periodic)
    SmartDashboard.putNumber("Velocity", m_motor.getVelocity( ));

    m_elevSim.periodic( );

    if (m_controller.getAButtonPressed( ))
    { // if A button pressed, set voltage of 0.3 // TODO: The code now says the exact same thing as the comment--delete the comment.
      m_pidEnabled = false;
      DataLogManager.log("A Button Pressed -- Set motor to constant forward speed (PID disabled)"); // TODO: Say the FUNCTION of what's happening (the value will be logged by .set())
      m_motor.set(kConstantOutput);
    }

    if (m_controller.getBButtonPressed( ))
    { // if B button pressed, set voltage of -0.3 // TODO: See above... delete this comment.
      m_pidEnabled = false;
      DataLogManager.log("B Button Pressed -- Set motor to constant reverse speed (PID disabled)");
      m_motor.set(-kConstantOutput);
    }

    if (m_controller.getXButtonPressed( ))
    { // if X button pressed, set PID at setpoint of 1.0  // TODO: See above... delete this comment.
     // m_motor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, 1.0, 0); // TODO: Looks like more old comments to delete!
     // DataLogManager.log("X Button Pressed -- PID Setpoint: 1.0");
     // goal = 4096.0;    // TODO: The whole point of putting the encoder conversions in ExampleSmartMotorController is to hide them from this "Robot.java" level
     //                             so we won't be using this "goal = 4096.0", because m_goal will be in rotations!
      m_pidEnabled = true;
      DataLogManager.log("X Button Pressed -- Trapezoid Profile Forward Goal (PID enabled)");
      m_goal = new TrapezoidProfile.State(kForwardGoal, 0); // test this value (position value :)
    }

    if (m_controller.getYButtonPressed( ))
    { // if Y button pressed, set PID at setpoint of 0.0  // TODO: More old comments to delete!
     // m_motor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, 0.0, 0);
     // DataLogManager.log("Y Button Pressed -- PID Setpoint: 0.0");
     // goal = 0.0;
      m_pidEnabled = true;
      m_goal = new TrapezoidProfile.State(kReverseGoal, 0); // test this value (position value :)
      DataLogManager.log("Y Button Pressed -- Trapezoid Profile Reverse Goal (PID enabled)");
    }

    if (m_controller.getRightBumperPressed( ))
    { // if Right Bumper pressed, stop Motor // TODO: More old comments to delete!
      m_pidEnabled = false;
      DataLogManager.log("Right Bumper Pressed -- Stop Motor (PID disabled)");
      m_motor.stopMotor( );
    }

    // TODO: Try to implement the simple set(get) structure we talked about for toggling the inversion

    // TODO: I think you'll find that getStartButtonPressed IS raw button 8--try to use that method
    if (m_controller.getRawButtonPressed(8))
    { // if menu button, invert motor direction
      m_pidEnabled = false;
      DataLogManager.log("Menu Button Pressed - Invert Motor Direction (PID disabled)"); // controller is not reading the button being pressed? // TODO: Is this comment still correct or useful?
      if (m_motor.getInverted( ))
      {
        m_motor.setInverted(false);
        DataLogManager.log("Motor Inverted -- False");
      }
      else
      {
        m_motor.setInverted(true);
        DataLogManager.log("Motor Inverted -- True");
      }
    }

    // TODO: 
    //        Since this runs every time, it will block all other motor commands (like set())
    //        We only want this to run when the X or Y button has been pressed, so maybe it makes 
    //        sense to have the X and Y buttons ENABLE this section of code through a boolean
    //        while this mode is active. Then clear that enable flag when you want to use other 
    //        modes
    //  if (m_pidEnabled)
    //    {}
    //
    // TODO: Although I see the m_pidEnabled variable set and cleared, it's not being used to control the following two lines properly
    m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
    m_motor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, m_setpoint.position, 0.0); // why divide by 12?

  }
}
