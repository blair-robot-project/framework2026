package frc.team449

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.RobotBase
import frc.team449.subsystems.drive.SwerveConstants
import kotlin.math.PI

object RobotConstants {
  enum class Mode {
    REAL,
    SIM,
    REPLAY
  }

  val CURRENT_MODE: Mode = if (RobotBase.isReal()) Mode.REAL else Mode.SIM

  const val TUNING_MODE: Boolean = false

  const val PDH_CAN_ID = 1

  /** Controller Configurations */
  val ROT_RATE_LIMIT: Double = 17.27 * PI // rad /s
  val NEG_ROT_RATE_LIM: Double = -27.5 * PI // rad / s
  const val DRIVE_DEADBAND = 0.1
  const val ROT_DEADBAND = 0.1
  val SNAP_TO_ANGLE_TOLERANCE_RAD: Angle = Units.Degrees.of(3.5)

  /** Drive Configuration */
  val MAX_LINEAR_SPEED = SwerveConstants.MAX_LINEAR_SPEED // m/s
  const val MAX_ROT_SPEED = 4.94967 * PI / 4 // rad/s

  const val USE_ACCEL_LIMIT = true

  val MAX_ACCEL = 25.0 // m/s/s
//    4 *
//    DCMotor.getNEO(1)
//      .getTorque(75.0) /
//    ((SwerveConstants.DRIVE_UPR / (2 * PI)) * ROBOT_WEIGHT * SwerveConstants.DRIVE_GEARING) // m/s/s

  val INITIAL_POSE = Pose2d(0.0, 0.0, Rotation2d())

  init {
    println("Drive Max Accel: $MAX_ACCEL")
  }

  const val LOOP_TIME = 0.020

  const val ROBOT_MASS_KG = 54.43
}
