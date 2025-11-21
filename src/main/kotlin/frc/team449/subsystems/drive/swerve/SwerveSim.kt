package frc.team449.subsystems.drive.swerve

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructPublisher
import edu.wpi.first.wpilibj.Timer.getFPGATimestamp
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation

class SwerveSim(
  frontLeftModule: SwerveModule,
  frontRightModule: SwerveModule,
  backLeftModule: SwerveModule,
  backRightModule: SwerveModule,
  maxLinearSpeed: Double,
  accel: Double,
  maxRotSpeed: Double,
  field: Field2d,
  maxModuleSpeed: Double,
  var driveSim: SwerveDriveSimulation
) : SwerveDrive(frontLeftModule, frontRightModule, backLeftModule, backRightModule, maxLinearSpeed, accel, maxRotSpeed, field, maxModuleSpeed) {

  private var lastTime = getFPGATimestamp()
  var currHeading = Rotation2d()

  private val odometryTracker = SwerveDriveOdometry(
    kinematics,
    currHeading,
    getPositions(),
    Pose2d()
  )

  var odometryPose: Pose2d = driveSim.simulatedDriveTrainPose

  private var maplesimDrive: StructPublisher<Pose2d> = NetworkTableInstance.getDefault()
    .getStructTopic<Pose2d>("Maplesim Drive", Pose2d.struct).publish()

  override fun periodic() {
    val currTime = getFPGATimestamp()
    this.lastTime = currTime

    set(super.desiredSpeeds)

    // Updates the robot's currentSpeeds.
    currentSpeeds = driveSim.driveTrainSimulatedChassisSpeedsFieldRelative

    // Update Robot Position
    currHeading = driveSim.simulatedDriveTrainPose.rotation
    odometryPose = driveSim.simulatedDriveTrainPose

    // Publish Maplesim Position
    maplesimDrive.set(driveSim.simulatedDriveTrainPose)
  }

  fun resetOdometryOnly(pose: Pose2d) {
    driveSim.setSimulationWorldPose(pose)
    odometryPose = pose
  }
}