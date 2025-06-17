package frc.team449.subsystems.superstructure.intake

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue

object IntakeConstants {
  // TODO: update the ids for all three kraken motors

  const val TOP_MOTOR_ID = 47
  const val RIGHT_MOTOR_ID = 46
  const val LEFT_MOTOR_ID = 48

//  val CURRENT_LIMIT: Current = Amps.of(30.0)

  val SUPPLY_LIM = 40.0
  val STATOR_LIM = 80.0

//  const val BRAKE_MODE = true

  const val MOTOR_INVERTED = false

  val INVERTED = InvertedValue.Clockwise_Positive
  val BRAKE_MODE = NeutralModeValue.Brake

  // voltage for different scenarios and motors
  const val TOP_ROLLER_VOLTAGE = 10.0 // TODO
  const val RUN_SIDES_TO_LEFT = 7.0
  const val RUN_SIDES_TO_RIGHT = 7.0

  const val CORAL_INTAKE_VOLTAGE = 11.0
  const val CORAL_INTAKE_VOLTAGE_LOWER = 2.0

// TODO: intake voltage for different scenarios and motors
  const val ALGAE_INTAKE_VOLTAGE = 9.0

  const val ALGAE_HOLD_VOLTAGE = 4.15
  const val CORAL_HOLD_VOLTAGE = 0.35

  const val CORAL_OUTTAKE_VOLTAGE = 3.0
  const val CORAL_OUTTAKE_PIVOT_SIDE_VOLTAGE = -3.0
  const val L1_OUTTAKE = -2.1678
  const val ALGAE_OUTTAKE_VOLTAGE = -10.0
  const val ALGAE_STALL_TRESHHOLD = 500.0

  const val DESCORE_ALGAE_VOLTAGE = -3.0

  const val WAIT_AFTER_ALGAE_DETECTED = 0.5

  const val ALGAE_SENSOR_DIO_PORT = 11
  const val PIVOT_SENSOR_DIO_PORT = 12

  const val READY_PIVOT_CORAL_TIME = 0.25

  // TODO Add CAN IDs for new LaserCAN sensors

  const val RIGHT_CORAL_SENSOR_CAN_ID = 20
  const val MIDDLE_CORAL_SENSOR_CAN_ID = 21
  const val LEFT_CORAL_SENSOR_CAN_ID = 22
  const val BACK_CORAL_SENSOR_CAN_ID = 23

  // Minimum distance in mm on the LaserCAN sensors to count as a detection (TODO Calibration)
  const val LASER_CAN_SENSOR_MIN_DISTANCE_MM = 64
  const val CORAL_DETECTION_THRESHOLD = 50
}
