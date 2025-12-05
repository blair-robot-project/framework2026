package frc.team449

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team449.commands.drive.WheelRadiusCharacterization
import frc.team449.config.RobotConstants
import frc.team449.hardwaremanagers.drive.swerve.SwerveSim
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI
import kotlin.random.Random

object Commands {

  fun resetGyro(): Command {
    return ConditionalCommand(
      InstantCommand({ Robot.poseSubsystem.heading = Rotation2d(PI) }),
      InstantCommand({ Robot.poseSubsystem.heading = Rotation2d() }),
    ) { DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red }
  }

  fun slowDrive(): Command {
    return InstantCommand({ Robot.holonomicOi.maxLinearSpeed = MetersPerSecond.of(1.0) })
      .andThen(InstantCommand({ Robot.holonomicOi.maxRotationalSpeed = RadiansPerSecond.of(PI / 2) }))
  }

  fun restoreDriveSpeed(): Command {
    return InstantCommand({ Robot.holonomicOi.maxLinearSpeed = RobotConstants.MAX_LINEAR_SPEED })
      .andThen(
        InstantCommand({ Robot.holonomicOi.maxRotationalSpeed = RobotConstants.MAX_ROT_SPEED }),
      )
  }

  fun resetOdometrySim(): Command {
    return InstantCommand({
      Robot.drive as SwerveSim
      Robot.drive.resetOdometryOnly(
        Pose2d(
          Robot.drive.odometryPose.x + Random.nextDouble(-1.0, 1.0),
          Robot.drive.odometryPose.y + Random.nextDouble(-1.0, 1.0),
          Robot.drive.odometryPose.rotation,
        )
      )
    })
  }

  fun pointToRight(): Command {
    return Robot.driveCommand.pointAtAngleCommand(Rotation2d.fromDegrees(90.0))
  }

  /** Characterization functions */
  fun wheelRadiusCharacterization(): Command {
    return WheelRadiusCharacterization(Robot.drive, Robot.poseSubsystem)
  }

  fun driveCharacterization(): SysIdRoutine {
    return SysIdRoutine(
      SysIdRoutine.Config(
        Volts.of(1.0).per(Second),
        Volts.of(2.0),
        Seconds.of(4.0),
      ) { state -> SignalLogger.writeString("state", state.toString()) },
      SysIdRoutine.Mechanism(
        { voltage: Voltage -> Robot.drive.setVoltage(-voltage.`in`(Volts)) },
        null,
        Robot.drive,
      )
    )
  }
}
