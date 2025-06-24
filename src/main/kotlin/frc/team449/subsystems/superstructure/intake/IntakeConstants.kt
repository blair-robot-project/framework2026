package frc.team449.subsystems.superstructure.intake

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue

object IntakeConstants {
  const val TOP_MOTOR_ID = 47
  const val RIGHT_MOTOR_ID = 46
  const val LEFT_MOTOR_ID = 48

  val TOP_INVERTED = InvertedValue.Clockwise_Positive
  val RIGHT_INVERTED = InvertedValue.CounterClockwise_Positive
  val LEFT_INVERTED = InvertedValue.Clockwise_Positive
  val BRAKE_MODE = NeutralModeValue.Brake

  const val SUPPLY_LIM = 40.0
  const val STATOR_LIM = 80.0

  // voltage for different scenarios and motors
  const val TOP_ROLLER_IN_VOLTAGE = 10.0
  const val SIDE_ROLLER_IN_VOLTAGE = 6.0

  const val TOP_ROLLER_OUT_VOLTAGE = -5.0
  const val SIDES_OUT_VOLTAGE = -3.0

  const val RUN_SIDES = 7.0
  const val CORAL_OUTTAKE_VOLTAGE = -3.0
  const val CORAL_OUTTAKE_PIVOT_SIDE_VOLTAGE = -3.0
  const val L1_OUTTAKE = -2.1678

  // TODO: all voltages for algae
  const val ALGAE_INTAKE_VOLTAGE = 9.0
  const val ALGAE_HOLD_VOLTAGE = 2.0
  const val ALGAE_OUTTAKE_VOLTAGE = -11.0
  const val ALGAE_STALL_THRESHOLD = 30.0

  const val DESCORE_ALGAE_VOLTAGE = -3.0
  const val WAIT_AFTER_ALGAE_DETECTED = 0.5
  const val ALGAE_SENSOR_DIO_PORT = 11
  const val PIVOT_SENSOR_DIO_PORT = 12
  const val READY_PIVOT_CORAL_TIME = 0.25

  const val HOLDING_FINISH_VELOCITY = 0.25
  const val CORAL_CENTER_WAIT_TIME = 0.1

  const val RIGHT_CORAL_SENSOR_CAN_ID = 20
  const val MIDDLE_CORAL_SENSOR_CAN_ID = 21
  const val LEFT_CORAL_SENSOR_CAN_ID = 22
  const val BACK_CORAL_SENSOR_CAN_ID = 23

  // Minimum distance in mm on the LaserCAN sensors to count as a detection (TODO Calibration)
  const val CORAL_DETECTION_THRESHOLD = 50
}
