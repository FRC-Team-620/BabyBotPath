// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.kauailabs.navx.frc.AHRS;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.RelativeEncoder;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.math.numbers.N2;
// import edu.wpi.first.math.system.LinearSystem;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.SerialPort.Port;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.interfaces.Gyro;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
// import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
// import edu.wpi.first.wpilibj.simulation.EncoderSim;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Robot;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Util.sim.NavxWrapper;
// import frc.robot.Util.sim.RevEncoderSimWrapper;

// public class DriveSubsystem extends SubsystemBase {
//   private final CANSparkMax leftMotorMaster = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
//   private final CANSparkMax leftMotorFollower = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);

//   private final CANSparkMax rightMotorMaster = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
//   private final CANSparkMax rightMotorFollower = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

//   // // The motors on the left side of the drive.
//   // private final MotorControllerGroup m_leftMotors = new
//   // MotorControllerGroup(leftMotor1, leftMotor2);
//   // // The motors on the right side of the drive.
//   // private final MotorControllerGroup m_rightMotors = new
//   // MotorControllerGroup(rightMotor1, rightMotor2);

//   // The robot's drive
//   private final DifferentialDrive m_drive = new DifferentialDrive(leftMotorMaster, rightMotorMaster);

//   // The left-side drive encoder
//   private final RelativeEncoder m_leftEncoder = leftMotorMaster.getEncoder();

//   // The right-side drive encoder
//   private final RelativeEncoder m_rightEncoder = rightMotorMaster.getEncoder();
//   // m_rightEncoder.setInverted(true);

//   // The gyro sensor
//   private final Gyro m_gyro = new AHRS(Port.kMXP);

//   // Odometry class for tracking robot pose
//   private final DifferentialDriveOdometry m_odometry;

//   private DifferentialDrivetrainSim m_drivetrainSimulator;
//   private RevEncoderSimWrapper leftencsim, rightencsim;
//   //private NavxWrapper s_gyro = new NavxWrapper();
//   // private

//   /** Creates a new DriveSubsystem. */
//   public DriveSubsystem() {
//     leftMotorMaster.restoreFactoryDefaults();
//     leftMotorFollower.restoreFactoryDefaults();
//     rightMotorMaster.restoreFactoryDefaults();
//     rightMotorFollower.restoreFactoryDefaults();
//     // We need to invert one side of the drivetrain so that positive voltages
//     // result in both sides moving forward. Depending on how your robot's
//     // gearbox is constructed, you might have to invert the left side instead.
//     leftMotorFollower.follow(leftMotorMaster, true); // TODO: Verifiy if the motors need to be inverted or not?
//     rightMotorFollower.follow(rightMotorMaster, true);// This inversion just makes the follower and master move in
//                                                       // different directions IE if they where in a gearbox. If
//                                                       // inversions are needed they should be done on the master motor
//                                                       // controler as this will also update the encoder outputs.
//     // m_rightMotors.setInverted(true);
//     // m_leftMotors.setInverted(false);
//     // leftMotor1.setInverted(true);
//     // leftMotor2.setInverted(true);
//     // rightMotor1.setInverted(true);
//     // rightMotor2.setInverted(true);
//     // Sets the distance per pulse for the encoders
//     m_leftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
//     m_rightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);

//     m_leftEncoder.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse);
//     m_rightEncoder.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse);

//     // resetEncoders();
//     m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
//     LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(
//         Constants.SimConstants.kSimDrivekVLinear,
//         Constants.SimConstants.ksimDrivekALinear, Constants.SimConstants.ksimDrivekVAngular,
//         Constants.SimConstants.kSimDrivekAAngular);
//     m_drivetrainSimulator = new DifferentialDrivetrainSim(
//         m_drivetrainSystem, DCMotor.getNEO(2), Constants.SimConstants.gearRatio,
//         Constants.DriveConstants.kTrackwidthMeters,
//         Constants.DriveConstants.kWheelDiameterMeters / 2, null);
//     this.leftencsim = RevEncoderSimWrapper.create(leftMotorMaster);
//     this.rightencsim = RevEncoderSimWrapper.create(rightMotorMaster);
//   }

//   @Override
//   public void periodic() {
//     // Update the odometry in the periodic block
//     m_odometry.update(
//         m_gyro.getRotation2d(), getLeftPosition(), getRightPosition());

//     pushOdometry();
//   }

//   public double getLeftPosition() {
//     return m_leftEncoder.getPosition();
//   }

//   public double getRightPosition() {
//     return m_rightEncoder.getPosition();
//   }

//   public double getLeftVelocity() {
//     return m_leftEncoder.getVelocity();
//   }

//   public double getRightVelocity() {
//     return m_rightEncoder.getVelocity();
//   }

//   private void pushOdometry() {
//     SmartDashboard.putNumber("L Encoder", getLeftPosition());
//     SmartDashboard.putNumber("R Encoder", getRightPosition());
//     SmartDashboard.putNumber("Gyro", getHeading());
//   }

//   /**
//    * Returns the currently-estimated pose of the robot.
//    *
//    * @return The pose.
//    */
//   public Pose2d getPose() {
//     return m_odometry.getPoseMeters();
//   }

//   /**
//    * Returns the current wheel speeds of the robot.
//    *
//    * @return The current wheel speeds.
//    */
//   public DifferentialDriveWheelSpeeds getWheelSpeeds() {
//     return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
//   }

//   /**
//    * Resets the odometry to the specified pose.
//    *
//    * @param pose The pose to which to set the odometry.
//    */
//   public void resetOdometry(Pose2d pose) {
//     resetEncoders();
//     m_drivetrainSimulator.setPose(pose);
//     m_odometry.resetPosition(pose, m_gyro.getRotation2d());
//   }

//   /**
//    * Drives the robot using arcade controls.
//    *
//    * @param fwd the commanded forward movement
//    * @param rot the commanded rotation
//    */
//   public void arcadeDrive(double fwd, double rot) {
//     m_drive.arcadeDrive(fwd, rot);
//   }

//   /**
//    * Controls the left and right sides of the drive directly with voltages.
//    *
//    * @param leftVolts  the commanded left output
//    * @param rightVolts the commanded right output
//    */
//   public void tankDriveVolts(double leftVolts, double rightVolts) {
//     if (Robot.isSimulation()) { // Limitation with simulation class
//       leftMotorMaster.set(leftVolts * RobotController.getInputVoltage());
//       rightMotorMaster.set(rightVolts * RobotController.getInputVoltage());
//     } else {
//       leftMotorMaster.setVoltage(leftVolts);
//       rightMotorMaster.setVoltage(rightVolts);
//     }
//     m_drive.feed();
//   }

//   /** Resets the drive encoders to currently read a position of 0. */
//   public void resetEncoders() {
//     m_leftEncoder.setPosition(0);
//     m_rightEncoder.setPosition(0);
//   }

//   /**
//    * Gets the average distance of the two encoders.
//    *
//    * @return the average of the two encoder readings
//    */
//   public double getAverageEncoderDistance() {
//     // return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
//     return (getLeftPosition() + getRightPosition()) / 2;
//   }

//   /**
//    * Gets the left drive encoder.
//    *
//    * @return the left drive encoder
//    */
//   public RelativeEncoder getLeftEncoder() {
//     return m_leftEncoder;
//   }

//   /**
//    * Gets the right drive encoder.
//    *
//    * @return the right drive encoder
//    */
//   public RelativeEncoder getRightEncoder() {
//     return m_rightEncoder;
//   }

//   /**
//    * Sets the max output of the drive. Useful for scaling the drive to drive more
//    * slowly.
//    *
//    * @param maxOutput the maximum output to which the drive will be constrained
//    */
//   public void setMaxOutput(double maxOutput) {
//     m_drive.setMaxOutput(maxOutput);
//   }

//   /** Zeroes the heading of the robot. */
//   public void zeroHeading() {
//     m_gyro.reset();
//   }

//   /**
//    * Returns the heading of the robot.
//    *
//    * @return the robot's heading in degrees, from -180 to 180
//    */
//   public double getHeading() {
//     return m_gyro.getRotation2d().getDegrees();
//   }

//   /**
//    * Returns the turn rate of the robot.
//    *
//    * @return The turn rate of the robot, in degrees per second
//    */
//   public double getTurnRate() {
//     return m_gyro.getRate();
//   }

//   @Override
//   public void simulationPeriodic() {
//     m_drivetrainSimulator.setInputs(
//         this.leftMotorMaster.get() * RobotController.getInputVoltage(),
//         this.rightMotorMaster.get() * RobotController.getInputVoltage());
//     // SmartDashboard.putNumber("left motor out", this.leftMotorMaster.get());
//     m_drivetrainSimulator.update(Constants.SimConstants.kSimUpdateTime);
//     this.leftencsim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
//     this.leftencsim.setVelocity(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());

//     this.rightencsim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
//     this.rightencsim.setVelocity(m_drivetrainSimulator.getRightVelocityMetersPerSecond());

//     //s_gyro.getYawGyro().setAngle(-m_drivetrainSimulator.getHeading().getDegrees()); // TODO add Gyo Vel support
//   }
// }
