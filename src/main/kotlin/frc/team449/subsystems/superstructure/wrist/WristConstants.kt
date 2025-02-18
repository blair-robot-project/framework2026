package frc.team449.subsystems.superstructure.wrist

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Frequency
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import kotlin.math.PI

object WristConstants {
  const val ANGLE = 0.0
  const val WIDTH = 2.0
  val COLOR = Color8Bit(Color.kWhite)

  const val MOTOR_ID = 41

  val TOLERANCE = Degrees.of(5.0)
  const val GEARING = 17.0 / 576.0
  const val UPR = 2 * PI

  val INVERTED = InvertedValue.Clockwise_Positive
  val BRAKE_MODE = NeutralModeValue.Brake

  const val STATOR_LIM = 80.0
  const val SUPPLY_LIM = 40.0

  val VALUE_UPDATE_RATE: Frequency = Hertz.of(50.0)
  val REQUEST_UPDATE_RATE: Frequency = Hertz.of(100.0)

  const val KS = 0.020
  const val KG = 0.25
  const val KV = 0.63027

  const val KP = 1.5119
  const val KI = 0.0
  const val KD = 0.0096482

  val CRUISE_VEL = RadiansPerSecond.of(2 * PI)
  val MAX_ACCEL = RadiansPerSecondPerSecond.of(6 * PI)

  val STARTUP_ANGLE = Degrees.of(90.0)

  val RESET_ENC_LIMIT = Degrees.of(0.05)

  /** Encoder Values */
  const val ABS_ENC_DIO_PORT = 1
  const val ABS_OFFSET = 0.0
  const val ENC_INVERTED = false
  val ABS_RANGE = Pair(-0.25, 0.75)
  const val ENC_RATIO = 2 * PI / (30.0 / 30.0)
  const val ENC_CPR = 2048
  const val SAMPLES_TO_AVERAGE = 127
}
