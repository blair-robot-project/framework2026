package frc.team449.commands.autoscoreCommands

import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.pathfinding.LocalADStar
import com.pathplanner.lib.trajectory.PathPlannerTrajectory
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.networktables.*
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.Robot
import frc.team449.commands.driveAlign.SimpleReefAlign
import frc.team449.subsystems.FieldConstants
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveDrive
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.min

class AutoScorePathfinder(private val robot: Robot, private val endPose: Pose2d, private val scoringReef: Boolean) {
  private var ADStar = LocalADStar()

  private val velXPub: DoublePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/pathfinder/pathVelocityX").publish()
  private val velYPub: DoublePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/pathfinder/pathVelocityY").publish()
  private val velRotationPub: DoublePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/pathfinder/pathRotation").publish()
  private val setpointPub: BooleanPublisher = NetworkTableInstance.getDefault().getBooleanTopic("/pathfinder/atSetpoint").publish()
  private val rotPub: BooleanPublisher = NetworkTableInstance.getDefault().getBooleanTopic("/pathfinder/atRotSetpoint").publish()
  private val autodistancePub: BooleanPublisher = NetworkTableInstance.getDefault().getBooleanTopic("/pathfinder/inPIDDistance").publish()
  private val admagpub: DoublePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/pathfinder/admag").publish()
  private val distpub: DoublePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/pathfinder/distance").publish()
  private lateinit var path: PathPlannerPath

  private var velocityX = 0.0
  private var velocityY = 0.0
  private var rotation = 0.0
  private val timer = Timer()
  private var inPIDDistance = false
  private var pidDistance = 0.75
  private val tolerance = 0.02
  private val rotTol = 0.01
  private val premoveDistance = 0.3
  private val reefCenter = FieldConstants.REEF_CENTER

  var atSetpoint = false
  var atPremoveDistance = false

  private var trajValid = true
  private var startTime = 0.0
  private var expectedTime = 0.0
  private lateinit var trajectory: PathPlannerTrajectory
  private var pathNull = true
  private var trajectoryNull = true

  private var xPIDSpeed = 0.0
  private var yPIDSpeed = 0.0
  private val pidOffsetTime = 0.05

  private var thetaController = ProfiledPIDController(6.5, 0.0, 0.0, TrapezoidProfile.Constraints(AutoScoreCommandConstants.MAX_ROT_SPEED, AutoScoreCommandConstants.MAX_ROT_ACCEL))
  private var xController = ProfiledPIDController(3.0, 0.0, 0.0, TrapezoidProfile.Constraints(AutoScoreCommandConstants.MAX_LINEAR_SPEED, AutoScoreCommandConstants.MAX_ACCEL))
  private var yController = ProfiledPIDController(3.0, 0.0, 0.0, TrapezoidProfile.Constraints(AutoScoreCommandConstants.MAX_LINEAR_SPEED, AutoScoreCommandConstants.MAX_ACCEL))
  var distance: Double = 100.0
  private var ADStarPower = 0.95
  private val ADStarDecrease = 0.04
  private val pushbackMultiply = 0.5

  private val speedTol: Double = 0.075
  private val speedTolRot: Double = PI / 16
  private val ffMinRadius: Double = 0.02
  private val ffMaxRadius: Double = 0.1

  fun runSetup() {
    timer.restart()
    val currentPose = robot.poseSubsystem.pose
    ADStar.setStartPosition(currentPose.translation)
    ADStar.setGoalPosition(endPose.translation)
    velXPub.set(velocityX)
    velYPub.set(velocityY)
    velRotationPub.set(rotation)
    distance = currentPose.translation.getDistance(endPose.translation)
    if (distance < pidDistance) {
      ADStarPower = (distance / pidDistance)
    }

    val fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(
      robot.drive.currentSpeeds.vxMetersPerSecond,
      robot.drive.currentSpeeds.vyMetersPerSecond,
      robot.drive.currentSpeeds.omegaRadiansPerSecond,
      currentPose.rotation
    )

    xController.reset(
      currentPose.x,
      fieldRelative.vxMetersPerSecond
    )
    yController.reset(
      currentPose.y,
      fieldRelative.vyMetersPerSecond
    )

    thetaController.reset(
      currentPose.rotation.radians,
      fieldRelative.omegaRadiansPerSecond)
    thetaController.enableContinuousInput(-PI, PI)

    xController.setTolerance(0.0, speedTol)
    yController.setTolerance(0.0, speedTol)
    thetaController.setTolerance(0.0, speedTolRot)

    thetaController.goal = TrapezoidProfile.State(endPose.rotation.radians, 0.0)
    xController.goal = TrapezoidProfile.State(endPose.x, 0.0)
    yController.goal = TrapezoidProfile.State(endPose.y, 0.0)
  }

  fun pathFind() {
    val currentPose = robot.poseSubsystem.pose
    ADStar.setStartPosition(currentPose.translation)
    val currentTime = timer.get()
    distance = currentPose.translation.getDistance(endPose.translation)
    if (distance < pidDistance) {
      inPIDDistance = true
      ADStarPower -= ADStarDecrease
      if (ADStarPower < 0) {
        ADStarPower = 0.0
      }
      if (distance < tolerance) {
        atSetpoint = true
      } else if (distance < premoveDistance) {
        atPremoveDistance = true
      }
    } else {
      inPIDDistance = false
    }

    if (!atSetpoint) {
      if (ADStar.isNewPathAvailable) {
        val newPath: PathPlannerPath? = ADStar.getCurrentPath(
          PathConstraints(
            AutoScoreCommandConstants.MAX_LINEAR_SPEED,
            AutoScoreCommandConstants.MAX_ACCEL,
            5 * 2 * Math.PI,
            RobotConstants.ROT_RATE_LIMIT
          ),
          GoalEndState(0.0, endPose.rotation)
        )
        if (newPath == null) {
          pathNull = true
        } else {
          path = newPath
          pathNull = false
        }
        if (!pathNull) {
          val pathList = path.pathPoses.toTypedArray<Pose2d>()
          val newTraj: PathPlannerTrajectory? = path.generateTrajectory(
            robot.drive.currentSpeeds,
            Rotation2d(MathUtil.angleModulus(currentPose.rotation.radians)),
            RobotConfig.fromGUISettings()
          )
          if (newTraj == null) {
            trajectoryNull = true
          } else {
            trajectory = newTraj
            trajectoryNull = false
          }
          if (!trajectoryNull) {
            expectedTime = trajectory.totalTimeSeconds
            trajValid = (pathList[0] != endPose)
            startTime = currentTime
          }
        }
      }
      if (!trajectoryNull && trajValid) {
        if(currentTime - startTime + pidOffsetTime*2 > expectedTime) {
          xController.goal = TrapezoidProfile.State(endPose.x, 0.0)
          yController.goal = TrapezoidProfile.State(endPose.y, 0.0)
        } else {
          trajectory.sample(currentTime - startTime + pidOffsetTime).pose.let {
            val speedScalar = hypot(robot.drive.currentSpeeds.vxMetersPerSecond, robot.drive.currentSpeeds.vyMetersPerSecond)
            xController.goal = TrapezoidProfile.State(it.x, speedScalar)
            yController.goal = TrapezoidProfile.State(it.y, speedScalar)
          }
        }
        val ffXScaler = MathUtil.clamp(
          (abs(currentPose.x-endPose.x) - ffMinRadius) / (ffMaxRadius - ffMinRadius),
          0.0,
          1.0
        )
        val ffYScaler = MathUtil.clamp(
          (abs(currentPose.y-endPose.y) - ffMinRadius) / (ffMaxRadius - ffMinRadius),
          0.0,
          1.0
        )
        xPIDSpeed = (xController.setpoint.velocity * ffXScaler + xController.calculate(currentPose.x, endPose.x))
        yPIDSpeed = (yController.setpoint.velocity * ffYScaler + yController.calculate(currentPose.y, endPose.y))
        trajectory.sample(currentTime - startTime).fieldSpeeds.let {
          val trajSpeeds = ChassisSpeeds(
            it.vxMetersPerSecond,
            it.vyMetersPerSecond,
            it.omegaRadiansPerSecond
          )
          velocityX = trajSpeeds.vxMetersPerSecond * ADStarPower
          velocityY = trajSpeeds.vyMetersPerSecond * ADStarPower
        }
      }
    }

    xPIDSpeed *= (1-ADStarPower)
    yPIDSpeed *= (1-ADStarPower)

    val ffRotScaler = MathUtil.clamp(
      (abs(MathUtil.angleModulus(currentPose.rotation.radians)-endPose.rotation.radians)) / (PI),
      0.0,
      1.0
    )
    rotation = thetaController.setpoint.velocity * ffRotScaler + thetaController.calculate(currentPose.rotation.radians, TrapezoidProfile.State(endPose.rotation.radians, 0.0))
    if (currentPose.rotation.radians == endPose.rotation.radians) {
      rotation = 0.0
    }

    val distanceToReef = currentPose.translation.getDistance(reefCenter)
    val reefPushbackTranslation = if(scoringReef && distanceToReef < 1.5 && distance > 0.6505 && currentTime-startTime < 1.0) {
      currentPose.translation.minus(reefCenter)*(distance-0.6505)*pushbackMultiply
    } else Translation2d(0.0, 0.0)

    if (atSetpoint) {
      robot.poseSubsystem.setPathMag(ChassisSpeeds(0.0, 0.0, rotation))
    } else {
      val fieldRelative = fromFieldRelativeSpeeds(
        ChassisSpeeds(
          MathUtil.clamp((velocityX + xPIDSpeed + reefPushbackTranslation.x), -AutoScoreCommandConstants.MAX_LINEAR_SPEED, AutoScoreCommandConstants.MAX_LINEAR_SPEED),
          MathUtil.clamp((velocityY + yPIDSpeed + reefPushbackTranslation.y), -AutoScoreCommandConstants.MAX_LINEAR_SPEED, AutoScoreCommandConstants.MAX_LINEAR_SPEED),
          MathUtil.clamp(rotation, -AutoScoreCommandConstants.MAX_ROT_SPEED, AutoScoreCommandConstants.MAX_ROT_SPEED)
        ),
        currentPose.rotation
      )
      robot.poseSubsystem.setPathMag(fieldRelative)
    }
    setpointPub.set(atSetpoint)
    rotPub.set(abs(currentPose.rotation.radians - endPose.rotation.radians) < rotTol)
    autodistancePub.set(inPIDDistance)
    velXPub.set(xPIDSpeed)
    velYPub.set(yPIDSpeed)
    velRotationPub.set(rotation)
    admagpub.set(ADStarPower)
    distpub.set(distance)
  }
}

class EmptyDrive(drive: SwerveDrive) : Command() {
  init {
    addRequirements(drive)
  }
  override fun isFinished(): Boolean {
    return true
  }
}

// goal should be a premove state
class AutoScoreWrapperCommand(
  private val robot: Robot,
  private val pathFinder: AutoScorePathfinder,
  private val premoveGoal: Command
) :
  Command() {

  private var hasPremoved = false

  override fun initialize() {
    robot.drive.defaultCommand.cancel()
    robot.drive.defaultCommand = EmptyDrive(robot.drive)
    robot.drive.defaultCommand.schedule()
    pathFinder.runSetup()
  }

  override fun execute() {
    if (!hasPremoved && pathFinder.atPremoveDistance) {
      premoveGoal.schedule()
      hasPremoved = true
    }
    if (!pathFinder.atSetpoint) {
      pathFinder.pathFind()
    }
  }

  override fun isFinished(): Boolean {
    return pathFinder.atSetpoint
  }
}
