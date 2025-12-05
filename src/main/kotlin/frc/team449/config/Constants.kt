package frc.team449.config

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.MetersPerSecondPerSecond
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.RadiansPerSecondPerSecond
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import org.ironmaple.simulation.drivesims.COTS
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig
import kotlin.math.PI

object RobotConstants {

  /** Other CAN ID */
  const val PDH_CAN = 1

  /** Controller Configurations */
  val ROT_RATE_LIMIT = RadiansPerSecondPerSecond.of(17.27 * PI)
  const val DRIVE_RADIUS_DEADBAND = .125
  const val ROTATION_DEADBAND = .125
  val SNAP_TO_ANGLE_TOLERANCE_RAD = Units.degreesToRadians(3.5)

  /** Drive Configuration */
  const val FIELD_RELATIVE_ENABLED = true
  val MAX_LINEAR_SPEED: LinearVelocity = SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED
  val MAX_ROT_SPEED: AngularVelocity = RadiansPerSecond.of(4.9496 * PI / 4)

  const val USE_ACCEL_LIMIT = true

  val MAX_ACCEL = MetersPerSecondPerSecond.of(25.0)

  val INITIAL_POSE = Pose2d(0.0, 0.0, Rotation2d())
  val MODULE_SIMULATION: SwerveModuleSimulationConfig = COTS.ofMark4(
    DCMotor.getKrakenX60(1),
    DCMotor.getKrakenX60(1),
    SwerveConstants.WHEEL_COF,
    SwerveConstants.GEAR_RATIO_LEVEL
  )

  const val LOOP_TIME = 0.020

  /** PID controller for snap to angle turning */
  val SNAP_KP = 5.85
  val SNAP_KI = 0.0
  val SNAP_KD = 0.0

  const val ALIGN_ROT_SPEED = 7 * PI / 2

  // Robot Dimensions (INCLUDING BUMPERS)
  val ROBOT_WIDTH = Units.inchesToMeters(27.25 + 3.25 * 2) // TODO: update for vetbot
  val ROBOT_LENGTH = Units.inchesToMeters(27.5 + 3.25 * 2) // TODO: update for vetbot
  val ROBOT_WEIGHT = 55.0 // TODO: update for vetbot
}

object FieldConstants {
  const val fieldLength = 17.55
  const val fieldWidth = 8.05

  private fun findPose(x: Double, y: Double, angle: Double, isRed: Boolean): Pose2d {
    return if (isRed) {
      Pose2d(fieldLength - x, fieldWidth - y, Rotation2d(MathUtil.angleModulus(angle + PI)))
    } else {
      Pose2d(x, y, Rotation2d(angle))
    }
  }
}

object LightConstants {
  const val LIGHT_PORT: Int = 9
  const val LIGHT_LENGTH: Int = 24

  // number of LEDs per meter (for WPILib calc)
  // todo(buscalo este numero)
  const val LED_PER_METER: Int = 120
  const val LED_TRANSLATION_SPEED: Double = 0.5 // m/s
}
