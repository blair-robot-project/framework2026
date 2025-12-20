package frc.team449.subsystems.drive

import edu.wpi.first.math.util.Units
import kotlin.math.PI

object SwerveConstants {
  const val EFFICIENCY = 0.95

  const val USE_FOC = false
  const val DUTY_CYCLE_DEADBAND = 0.001

  /** Drive CAN IDs */
  const val DRIVE_MOTOR_FL = 21
  const val DRIVE_MOTOR_FR = 22
  const val DRIVE_MOTOR_BL = 23
  const val DRIVE_MOTOR_BR = 24

  /** Turn CAN IDs */
  const val TURN_MOTOR_FL = 19
  const val TURN_MOTOR_FR = 6
  const val TURN_MOTOR_BL = 13
  const val TURN_MOTOR_BR = 5

  /** Turning encoder channels */
  const val TURN_ENC_CHAN_FL = 9
  const val TURN_ENC_CHAN_FR = 7
  const val TURN_ENC_CHAN_BL = 6
  const val TURN_ENC_CHAN_BR = 8

  // TODO: HI!
  const val PIGEON_CAN_ID = 0

  /** Offsets for the absolute encoders in rotations. */
  val TURN_ENC_OFFSET_FL =
    Units.radiansToRotations(-1.9721847889188047) +
      Units.radiansToRotations(-0.023566500800433245)
  val TURN_ENC_OFFSET_FR =
    Units.radiansToRotations(-1.3803421761481829) +
      Units.radiansToRotations(-1.4175450743616982) + 0.5
  val TURN_ENC_OFFSET_BL =
    Units.radiansToRotations(-0.8920550992085665) +
      Units.radiansToRotations(-1.9177244935091542 + 3.114585873128222)
  val TURN_ENC_OFFSET_BR =
    Units.radiansToRotations(-1.7617422152440068) +
      Units.radiansToRotations(-2.2696186936648175 - 0.8904340373587881) + 0.5

  /** Inversions */
  const val DRIVE_INVERTED = false
  const val TURN_INVERTED = true
  const val TURN_ENC_INVERTED = false

  /** Drive Gains */
  const val DRIVE_KP = 0.75
  const val DRIVE_KI = 0.0
  const val DRIVE_KD = 0.0
  const val DRIVE_KS = 0.15
  const val DRIVE_KV = 2.54
  const val DRIVE_KA = 0.47044

  /** Turn Gains */
  const val TURN_KP = 0.5
  const val TURN_KI = 0.0
  const val TURN_KD = 0.0
  const val TURN_KS = 0.05 / 12.0

  // drive config
  const val WHEEL_RADIUS_METERS = 0.049149 // m (1.935 in)
  const val DRIVE_UPR: Double = 2 * WHEEL_RADIUS_METERS * PI

  const val WHEEL_COF = 1.2

  const val DRIVE_L2_GEARING: Double = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) // 1/6.75
  const val DRIVE_L3_GEARING: Double = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0) // 1/6.12

  const val TURN_UPR: Double = 2 * PI
  const val MAX_LINEAR_SPEED: Double = 4.7244 // m / s (12 - DRIVE_KS) / DRIVE_KV
  const val MAX_ROT_SPEED: Double = 4.9496 * PI / 4

  const val DRIVE_SUPPLY_LIMIT: Double = 60.0 // amps
  const val DRIVE_STATOR_LIMIT: Double = 105.0 // amps
  const val STEERING_CURRENT_LIM: Double = 40.0 // amps

  val KRAKEN_UPDATE_RATE = 100.0 // hertz
  val VALUE_UPDATE_RATE = 50.0 // hertz

  const val JOYSTICK_FILTER_ORDER = 2
  const val ROT_FILTER_ORDER = 1.25
  const val SKEW_CONSTANT = 15.5

  /** Wheelbase = wheel-to-wheel distance from front to back of the robot */

  /** Trackwidth = wheel-to-wheel distance from side to side of the robot */
  val WHEELBASE = Units.inchesToMeters(27.0 - 5.25) // ex. FL to BL, aka 5.25in less than robot length
  val TRACKWIDTH = Units.inchesToMeters(27.0 - 5.25) // ex. BL to BR, aka 5.25in less than robot width
}
