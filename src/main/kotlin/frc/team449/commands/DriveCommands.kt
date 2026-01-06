package frc.team449.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.team449.Constants
import frc.team449.subsystems.drive.custom.SwerveConstants
import frc.team449.subsystems.drive.custom.SwerveDrive
import java.util.function.DoubleSupplier
import kotlin.math.hypot
import kotlin.math.withSign

// 1678: https://github.com/frc1678/C2025-Public/blob/main/src/main/java/frc/robot/controlboard/ControlBoard.java
// 6328: https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/main/src/main/java/org/littletonrobotics/frc2025/commands/DriveCommands.java

object DriveCommands {
  fun getLinearVelocityFromJoysticks(x: Double, y: Double): Translation2d {
    // apply deadband
    var linearMagnitude = MathUtil.applyDeadband(hypot(x, y), Constants.DriveConstants.TRANSLATION_DEADBAND)
    val linearDirection = if (hypot(x, y) > 1e-6) {
      Rotation2d(x, y)
    } else {
      Rotation2d() // default to 0 degrees without error
    }

    // square magnitude for more precise control
    linearMagnitude *= linearMagnitude

    // return new linear velocity
    return Pose2d(Translation2d.kZero, linearDirection)
      .transformBy(Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
      .translation
  }

  fun getOmegaFromJoysticks(driverOmega: Double): Double {
    val omega = MathUtil.applyDeadband(driverOmega, Constants.DriveConstants.ANGULAR_DEADBAND)
    return (omega * omega).withSign(omega)
  }

  fun joystickDrive(
    drive: SwerveDrive,
    xSupplier: DoubleSupplier,
    ySupplier: DoubleSupplier,
    omegaSupplier: DoubleSupplier
  ): Command {
    return Commands.run(
      {
        // get linear velocity
        val linearVelocity = getLinearVelocityFromJoysticks(xSupplier.asDouble, ySupplier.asDouble)
        val omega = getOmegaFromJoysticks(omegaSupplier.asDouble)

        // convert to field relative speeds & send command
        val speeds =
          ChassisSpeeds(
            linearVelocity.x * SwerveConstants.MAX_LINEAR_SPEED,
            linearVelocity.y * SwerveConstants.MAX_LINEAR_SPEED,
            omega * SwerveConstants.MAX_ROT_SPEED,
          )

        val isFlipped = DriverStation.getAlliance().isPresent && DriverStation.getAlliance().get() == Alliance.Red

        drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            if (isFlipped) {
              drive.rotation.plus(Rotation2d(Math.PI))
            } else {
              drive.rotation
            }
          )
        )
      },
      drive
    )
  }
}
