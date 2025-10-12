package frc.team449.subsystems.drive.swerve

import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.wpilibj.Timer.getFPGATimestamp
import edu.wpi.first.wpilibj.smartdashboard.Field2d


class SwerveSim(
  frontLeftModule: SwerveModule,
  frontRightModule: SwerveModule,
  backLeftModule: SwerveModule,
  backRightModule: SwerveModule,
  maxLinearSpeed: Double,
  accel: Double,
  maxRotSpeed: Double,
  field: Field2d,
  maxModuleSpeed: Double
) : SwerveDrive(frontLeftModule, frontRightModule, backLeftModule, backRightModule, maxLinearSpeed, accel, maxRotSpeed, field, maxModuleSpeed) {

  private var lastTime = getFPGATimestamp()
  var currHeading = Rotation2d()

  private val odometryTracker = SwerveDriveOdometry(
    kinematics,
    currHeading,
    getPositions(),
    Pose2d()
  )

  var odometryPose: Pose2d = odometryTracker.poseMeters

  override fun periodic() {
    val currTime = getFPGATimestamp()

    currHeading = currHeading.plus(Rotation2d(super.desiredSpeeds.omegaRadiansPerSecond * (currTime - lastTime)))
    this.lastTime = currTime

    set(super.desiredSpeeds)

    // Updates the robot's currentSpeeds.
    currentSpeeds = kinematics.toChassisSpeeds(
        frontLeftModule.state,
        frontRightModule.state,
        backLeftModule.state,
        backRightModule.state
    )

    odometryPose = odometryTracker.update(
      currHeading,
      getPositions()
    )

  }

  fun resetOdometryOnly(pose: Pose2d) {
    odometryTracker.resetPosition(
      currHeading,
      getPositions(),
      pose
    )
  }
}
