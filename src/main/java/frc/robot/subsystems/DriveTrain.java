// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;

import static frc.robot.Constants.*;
import frc.robot.Constants.DriveConstants;
import frc.robot.sim.PhysicsSim;;

public class DriveTrain extends SubsystemBase {
  // Declare all the member variables between here and the Constructor
  // Objects needed for Driving
  private Timer timer1;
  private WPI_TalonSRX m_right, m_left;
  private DifferentialDrive m_safetyDrive;
  private Encoder m_leftEncoder, m_rightEncoder;
  private ADXRS450_Gyro m_gyro;

  ////////////////////////////// Simulation ////////////////////
  private Field2d m_field;
  private DifferentialDrivetrainSim m_driveTrainSim;
  private EncoderSim m_leftEncoderSim, m_rightEncoderSim;
  private ADXRS450_GyroSim m_gyrosim;
  private DifferentialDriveOdometry m_odometry;

  // Constructor
  /** Creates a new Drivetrain, instantiating all the member variables. */
  public DriveTrain() {
    // Start the timer running from when the DT is instantiated
    timer1 = new Timer();
    timer1.start();

    // Setup the Talons and the safety drive for the robot to move,
    // use the same talon CAN IDs as our other bots
    m_left = new WPI_TalonSRX(LEFT_TALON_LEADER);
    m_right = new WPI_TalonSRX(RIGHT_TALON_LEADER);
    m_safetyDrive = new DifferentialDrive(m_left, m_right);
    // Set the talons to default configuration values
    m_right.configFactoryDefault();
    m_left.configFactoryDefault();
    m_right.setSensorPhase(true); // Invert the encoder on the right side

    m_leftEncoder = new Encoder(DriveConstants.kLeftEncoderPorts[0],
      DriveConstants.kLeftEncoderPorts[1], DriveConstants.kLeftEncoderReversed);
    m_rightEncoder = new Encoder(DriveConstants.kRightEncoderPorts[0],
      DriveConstants.kRightEncoderPorts[1], DriveConstants.kRightEncoderReversed);
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_leftEncoder.reset();
    m_rightEncoder.reset();

    m_gyro = new ADXRS450_Gyro();

    // Setup the robot for Simulation
    if(RobotBase.isSimulation()){
      // This is true, and it means we are doing the simulation not a real robot
      m_driveTrainSim = new DifferentialDrivetrainSim(DriveConstants.kDrivetrainPlant,
      DriveConstants.kDriveGearbox, DriveConstants.kDriveGearing, DriveConstants.kTrackwidthMeters,
      DriveConstants.kWheelDiameterMeters / 2.0, VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

      m_leftEncoderSim = new EncoderSim(m_leftEncoder);
      m_rightEncoderSim = new EncoderSim(m_rightEncoder);
      
      m_gyrosim = new ADXRS450_GyroSim(m_gyro);

      m_field = new Field2d();
      SmartDashboard.putData("Field", m_field);

      // Add the motor controllers to the Physics Engine
      PhysicsSim.getInstance().addTalonSRX(m_right, 0.75, 4000);
      PhysicsSim.getInstance().addTalonSRX(m_left, .75, 4000);

      m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    } // No more simulation constructs

    // This ends the DriveTrain Constructor
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // This method will be called once per scheduler run during simulation
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

    // Get the pose from the drive train and send it to the simulated field.
    m_field.setRobotPose(getPose());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_driveTrainSim.setInputs(
        m_left.get() * RobotController.getBatteryVoltage(),
        -m_right.get() * RobotController.getBatteryVoltage());
    m_driveTrainSim.update(0.020);

    m_leftEncoderSim.setDistance(m_driveTrainSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveTrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveTrainSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveTrainSim.getRightVelocityMetersPerSecond());
    m_gyrosim.setAngle(-m_driveTrainSim.getHeading().getDegrees());
  }

    /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the current being drawn by the drivetrain. This works in SIMULATION ONLY! If you want
   * it to work elsewhere, use the code in {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
   *
   * @return The drawn current in Amps.
   */
  public double getDrawnCurrentAmps() {
    return m_driveTrainSim.getCurrentDrawAmps();
  }

  public void manualDrive( double move, double turn){
    //m_safetyDrive.curvatureDrive(move, turn, (move == 0) ? true : false);
    m_safetyDrive.arcadeDrive(move, turn);
    m_safetyDrive.feed();
  }
}
