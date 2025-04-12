package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import choreo.trajectory.SwerveSample
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.*
import frc.team449.Robot
import frc.team449.commands.Commands.ScoreL4
import frc.team449.commands.driveAlign.SimpleReefAlign
import frc.team449.subsystems.FieldConstants
import frc.team449.subsystems.superstructure.SuperstructureGoal
import java.util.Optional

open class Routines(
  val robot: Robot
) {

  val autoFactory = AutoFactory(
    robot.poseSubsystem::pose,
    robot.poseSubsystem::resetOdometry,
    { sample: SwerveSample -> robot.drive.followTrajectory(robot, sample) },
    true,
    robot.drive
  )

  // do nothing
  fun doNothing(): AutoRoutine {
    val nothing: AutoRoutine = autoFactory.newRoutine("Nothing")
    return nothing
  }

  /** link to starting position on the field: https://docs.google.com/document/d/1SOzIJDgJ0GRSVnNTcBhaFfltvHw0IjJTEUsAZbI2hW4/edit?usp=sharing  **/
  /** left and right are from the driver's pov **/

  fun taxi(): AutoRoutine {
    val rTaxi: AutoRoutine = autoFactory.newRoutine("Right Taxi")
    val rTaxiTrajectory: AutoTrajectory = rTaxi.trajectory("prev/taxiRight")
    rTaxi.active().onTrue(Commands.sequence(rTaxiTrajectory.resetOdometry(), rTaxiTrajectory.cmd(), robot.drive.driveStop()))
    return rTaxi
  }
  fun middleRoutine(): AutoRoutine {
    val middleRoutine = autoFactory.newRoutine("one l4 ")
    val forward = middleRoutine.trajectory("OneL4/1")
    val end = middleRoutine.trajectory("OneL4/2")

    middleRoutine.active().onTrue(
      Commands.sequence(
        forward.resetOdometry(),
        forward.cmd().alongWith(
          WaitCommand(0.5).andThen(
            robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)
          )
        )
      )
    )

    forward.done().onTrue(
      Commands.sequence(
        ScoreL4(robot, FieldConstants.ReefSide.LEFT),
        end.cmd().alongWith( robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)),
        robot.drive.driveStop()
      )

    )
    return middleRoutine
  }


  /**Ground Intake Autos**/

/******
   uncomment all the
"//.onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }"
while testing on a real robot
   *****/

  // three l4 starting from a side then the back two reefs then half
  fun rightGround3L4Half(): AutoRoutine {
    val ground3halfRight = autoFactory.newRoutine("3 l4 and half")
    val preloadScore = ground3halfRight.trajectory("GroundThreeHalf/1r")
    val firstPickup = ground3halfRight.trajectory("GroundThreeHalf/2r")
    val firstPresagedScore = ground3halfRight.trajectory("GroundThreeHalf/3r")
    val secondPickup = ground3halfRight.trajectory("GroundThreeHalf/4r")
    val secondPresagedScore = ground3halfRight.trajectory("GroundThreeHalf/5r")
    val thirdPickup = ground3halfRight.trajectory("GroundThreeHalf/6r")
    val thirdPresagedScore = ground3halfRight.trajectory("GroundThreeHalf/7r")

    ground3halfRight.active().onTrue(
      Commands.sequence(
        preloadScore.resetOdometry().alongWith(robot.intake.stop()),
        preloadScore.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            .withDeadline(WaitCommand(1.5))
        )
      )
    )

    preloadScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.LEFT),
        firstPickup.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),
        (
          firstPresagedScore.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )
          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )

    firstPresagedScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.RIGHT),
        secondPickup.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),
        (
          secondPresagedScore.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )
          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )

    secondPresagedScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.LEFT),
        GroundIntake(robot).alongWith(
          thirdPickup.cmd()
        )
      )
    )

    thirdPickup.done().onTrue(
      robot.drive.driveStop()
    )

    return ground3halfRight
  }
  fun leftGround3L4Half(): AutoRoutine {
    val ground3halfLeft = autoFactory.newRoutine("3 l4 and half")
    val preloadScore = ground3halfLeft.trajectory("GroundThreeHalf/1l")
    val firstPickup = ground3halfLeft.trajectory("GroundThreeHalf/2l")
    val firstPresagedScore = ground3halfLeft.trajectory("GroundThreeHalf/3l")
    val secondPickup = ground3halfLeft.trajectory("GroundThreeHalf/4l")
    val secondPresagedScore = ground3halfLeft.trajectory("GroundThreeHalf/5l")
    val thirdPickup = ground3halfLeft.trajectory("GroundThreeHalf/6l")

    ground3halfLeft.active().onTrue(
      Commands.sequence(
        preloadScore.resetOdometry().alongWith(robot.intake.stop()),
        preloadScore.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            .withDeadline(WaitCommand(1.5))
        )
      )
    )

    preloadScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.RIGHT),
        firstPickup.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),
        (
          firstPresagedScore.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )
          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )

    firstPresagedScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.LEFT),
        secondPickup.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),
        (
          secondPresagedScore.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )
          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )

    secondPresagedScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.RIGHT),
        GroundIntake(robot).alongWith(
          thirdPickup.cmd()
        )
      )
    )

    thirdPickup.done().onTrue(
      robot.drive.driveStop()
    )

    return ground3halfLeft
  }


  // back l4 and then sides 2 l4
  fun left3L4(): AutoRoutine {
    val middlesides = autoFactory.newRoutine("3 l4")
    val preloadScore = middlesides.trajectory("middleSides/1l")
    val firstPickup = middlesides.trajectory("middleSides/2l")
    val firstPresagedScore = middlesides.trajectory("middleSides/3l")
    val secondPickup = middlesides.trajectory("middleSides/4l")
    val secondPresagedScore = middlesides.trajectory("middleSides/5l")
    val end = middlesides.trajectory("middleSides/endl")

    middlesides.active().onTrue(
      Commands.sequence(
        preloadScore.resetOdometry().alongWith(robot.intake.stop()),
        preloadScore.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            .withDeadline(WaitCommand(1.5))
        )
      )
    )

    preloadScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.LEFT),
        firstPickup.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),
        (
          firstPresagedScore.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )
          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )

    firstPresagedScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.RIGHT),
        secondPickup.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),
        (
          secondPresagedScore.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )
          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )
    secondPresagedScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.LEFT),
        end.cmd().alongWith( robot.superstructureManager.requestGoal(SuperstructureGoal.STOW))
      )
    )

    return middlesides
  }
  fun right3L4(): AutoRoutine {
    val middlesides = autoFactory.newRoutine("3 l4")
    val preloadScore = middlesides.trajectory("middleSides/1r")
    val firstPickup = middlesides.trajectory("middleSides/2r")
    val firstPresagedScore = middlesides.trajectory("middleSides/3r")
    val secondPickup = middlesides.trajectory("middleSides/4r")
    val secondPresagedScore = middlesides.trajectory("middleSides/5r")
    val end = middlesides.trajectory("middleSides/endr")


    middlesides.active().onTrue(
      Commands.sequence(
        preloadScore.resetOdometry().alongWith(robot.intake.stop()),
        preloadScore.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            .withDeadline(WaitCommand(1.5))
        )
      )
    )

    preloadScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.RIGHT),
        firstPickup.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),
        (
          firstPresagedScore.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )
          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )

    firstPresagedScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.LEFT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
        secondPickup.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),
        (
          secondPresagedScore.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )
          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )
    secondPresagedScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.RIGHT),
        end.cmd().alongWith( robot.superstructureManager.requestGoal(SuperstructureGoal.STOW))
      )
    )

    return middlesides
  }

  // two l4 and two l2 on the back reef branches
  fun rightGroundBack2L4l2(): AutoRoutine {
    val rightBack2l4l2 = autoFactory.newRoutine("2 l4 and l2")
    val scorePreloadB = rightBack2l4l2.trajectory("TwoL4L2/1r")
    val pickupMiddle = rightBack2l4l2.trajectory("TwoL4L2/2r")
    val scoreMiddleA = rightBack2l4l2.trajectory("TwoL4L2/3r")
    val pickupLeft = rightBack2l4l2.trajectory("TwoL4L2/4r")
    val scoreRightB = rightBack2l4l2.trajectory("TwoL4L2/5r")
    val pickupRight = rightBack2l4l2.trajectory("TwoL4L2/6r")
    val scoreLeftA = rightBack2l4l2.trajectory("TwoL4L2/7r")
    val end = rightBack2l4l2.trajectory("TwoL4L2/endr")

    rightBack2l4l2.active().onTrue(
      Commands.sequence(
        scorePreloadB.resetOdometry().alongWith(robot.intake.stop()),
        scorePreloadB.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            .withDeadline(WaitCommand(1.5))
        )
      )
    )

    scorePreloadB.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.LEFT),
        pickupMiddle.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),
        (
          scoreMiddleA.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )

          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )

    scoreMiddleA.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.RIGHT),
        pickupLeft.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),

        (
          scoreRightB.cmd().alongWith(
            WaitCommand(0.74).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L2_PREMOVE_PIVOT)
            )
          )
          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )

    scoreRightB.done()
      .onTrue(
        Commands.sequence(
          ScoreL2PivotSide(robot, FieldConstants.ReefSide.RIGHT),
          pickupRight.cmd().alongWith(GroundIntake(robot)),
          robot.drive.driveStop(),
          (
            scoreLeftA.cmd().alongWith(
              WaitCommand(0.68).andThen(
                robot.superstructureManager.requestGoal(SuperstructureGoal.L2_PREMOVE_PIVOT)
              )
            )
            ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
        )
      )

    scoreLeftA.done()
      .onTrue(
        Commands.sequence(
          ScoreL2PivotSide(robot, FieldConstants.ReefSide.LEFT),
          end.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)),
          robot.drive.driveStop(),

        )
      )

    return rightBack2l4l2
  }
  fun leftGroundBack2L4l2(): AutoRoutine {
    val leftBack2l4l2 = autoFactory.newRoutine("2 l4 and l2")
    val scorePreloadB = leftBack2l4l2.trajectory("TwoL4L2/1l")
    val pickupMiddle = leftBack2l4l2.trajectory("TwoL4L2/2l")
    val scoreMiddleA = leftBack2l4l2.trajectory("TwoL4L2/3l")
    val pickupLeft = leftBack2l4l2.trajectory("TwoL4L2/4l")
    val scoreRightB = leftBack2l4l2.trajectory("TwoL4L2/5l")
    val pickupRight = leftBack2l4l2.trajectory("TwoL4L2/6l")
    val scoreLeftA = leftBack2l4l2.trajectory("TwoL4L2/7l")
    val end = leftBack2l4l2.trajectory("TwoL4L2/endl")


    leftBack2l4l2.active().onTrue(
      Commands.sequence(
        scorePreloadB.resetOdometry().alongWith(
          robot.intake.stop()
        ),
        scorePreloadB.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            .withDeadline(WaitCommand(1.5))
        )
      )
    )

    scorePreloadB.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.LEFT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
        pickupMiddle.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),
        (
          scoreMiddleA.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )

          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )

    scoreMiddleA.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.RIGHT),
        pickupLeft.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),

        (
          scoreRightB.cmd().alongWith(
            WaitCommand(0.74).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )
          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )

    scoreRightB.done()
      .onTrue(
        Commands.sequence(
          ScoreL2PivotSide(robot, FieldConstants.ReefSide.RIGHT),
          pickupRight.cmd().alongWith(GroundIntake(robot)),
          robot.drive.driveStop(),
          (
            scoreLeftA.cmd().alongWith(
              WaitCommand(0.68).andThen(
                robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
              )
            )
            ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
        )
      )

    scoreLeftA.done()
      .onTrue(
        Commands.sequence(
          ScoreL2PivotSide(robot, FieldConstants.ReefSide.LEFT),
          end.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)),
          robot.drive.driveStop(),

        )
      )

    return leftBack2l4l2
  }



  fun noAlignLeftBack2L4l2(): AutoRoutine {
    val leftBack2l4l2 = autoFactory.newRoutine("2 l4 and l2")
    val scorePreloadB = leftBack2l4l2.trajectory("noAlignTwoL4L2/1l")
    val pickupMiddle = leftBack2l4l2.trajectory("noAlignTwoL4L2/2l")
    val scoreMiddleA = leftBack2l4l2.trajectory("noAlignTwoL4L2/3l")
    val pickupLeft = leftBack2l4l2.trajectory("noAlignTwoL4L2/4l")
    val scoreRightB = leftBack2l4l2.trajectory("noAlignTwoL4L2/5l")
    val pickupRight = leftBack2l4l2.trajectory("noAlignTwoL4L2/6l")
    val scoreLeftA = leftBack2l4l2.trajectory("noAlignTwoL4L2/7l")
    val end = leftBack2l4l2.trajectory("noAlignTwoL4L2/endl")


    leftBack2l4l2.active().onTrue(
      Commands.sequence(
        scorePreloadB.resetOdometry().alongWith(
          robot.intake.stop()
        ),
        scorePreloadB.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            .withDeadline(WaitCommand(1.5))
        )
      )
    )

    scorePreloadB.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSideN(robot),
        robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
        pickupMiddle.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),
        (
          scoreMiddleA.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )

          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )

    scoreMiddleA.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSideN(robot),
        robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
        pickupLeft.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),

        (
          scoreRightB.cmd().alongWith(
            WaitCommand(0.74).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L2_PREMOVE_PIVOT)
            )
          )
          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )

    scoreRightB.done()
      .onTrue(
        Commands.sequence(
          ScoreL2PivotSideN(robot),
          pickupRight.cmd().alongWith(GroundIntake(robot)),
          robot.drive.driveStop(),
          (
            scoreLeftA.cmd().alongWith(
              WaitCommand(0.68).andThen(
                robot.superstructureManager.requestGoal(SuperstructureGoal.L2_PREMOVE_PIVOT)
              )
            )
            ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
        )
      )

    scoreLeftA.done()
      .onTrue(
        Commands.sequence(
          ScoreL2PivotSideN(robot),
          end.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)),
          robot.drive.driveStop(),

          )
      )

    return leftBack2l4l2
  }

  fun american_routine_optimal(): AutoRoutine {
    val opt_american = autoFactory.newRoutine("opt Ameriacn")
    val l4A_traj = opt_american.trajectory("GroundThreeHalf/L4A (I)")
    val l4B_traj = opt_american.trajectory("GroundThreeHalf/l4B")
    val loli1_traj = opt_american.trajectory("GroundThreeHalf/Loli 1")
    val loli2_traj = opt_american.trajectory("GroundThreeHalf/Loli 2")
    val l3_traj = opt_american.trajectory("GroundThreeHalf/l3B")
    val loli3_traj = opt_american.trajectory("GroundThreeHalf/Loli 3")

    opt_american.active().onTrue(
      l4A_traj.resetOdometry().andThen(l4A_traj.cmd())

    )

    l4A_traj.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.LEFT),
        loli1_traj.cmd().alongWith(GroundIntake(robot)),
        l4B_traj.cmd(),
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.RIGHT),
        GroundIntake(robot).alongWith(loli2_traj.cmd()),
        l3_traj.cmd().andThen(ScoreL2PivotSide(robot, FieldConstants.ReefSide.RIGHT)),
        loli3_traj.cmd().alongWith(GroundIntake(robot))

      )
    )

    return opt_american
  }

  // Elevator is cooked!
  // autoChooser that will be displayed on dashboard
  fun addOptions(autoChooser: AutoChooser) {
    autoChooser.addRoutine("right 3.5 L4", this::rightGround3L4Half)
    autoChooser.addRoutine("left 3.5 fL4", this::leftGround3L4Half)

    autoChooser.addRoutine("RightBackL4+L2", this::rightGroundBack2L4l2)
    autoChooser.addRoutine("LeftBackL4+L2", this::leftGroundBack2L4l2)

    autoChooser.addRoutine("No align LeftBackL4+L2", this::noAlignLeftBack2L4l2)

    autoChooser.addRoutine("Left Back&Sides l4", this::left3L4)
    autoChooser.addRoutine("Right Middle&Sides", this::right3L4)

    autoChooser.addRoutine("Taxi", this::taxi)
     autoChooser.addRoutine("center one l4", this::middleRoutine)
  }

  fun ScoreL4PivotSide(robot: Robot, reefSide: FieldConstants.ReefSide): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PIVOT)
      .alongWith(
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(reefSide), translationSpeedLim = 1.0, translationAccelLim = 1.95)
          .andThen(PrintCommand("Actually reached auto tolerance!"))
          .withTimeout(2.0)
      )
      .andThen(WaitCommand(0.10))
      .andThen(robot.intake.outtakeCoralPivot())
      .andThen(
        WaitUntilCommand { !robot.intake.coralDetected() }
          .onlyIf { RobotBase.isReal() }
      )
      .andThen(WaitCommand(0.050))
      .andThen(robot.intake.stop())
  }


  fun ScoreL4PivotSideN(robot: Robot): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PIVOT)
      .andThen(WaitCommand(0.10))
      .andThen(robot.intake.outtakeCoralPivot())
      .andThen(
        WaitUntilCommand { !robot.intake.coralDetected() }
          .onlyIf { RobotBase.isReal() }
      )
      .andThen(WaitCommand(0.050))
      .andThen(robot.intake.stop())
  }

  fun ScoreL3PivotSide(robot: Robot, reefSide: FieldConstants.ReefSide): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.L3)
      .alongWith(
        // robot.intake.outtakeAlgae(),
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(reefSide), translationSpeedLim = 1.0, translationAccelLim = 1.95)
          .andThen(PrintCommand("Actually reached auto tolerance!"))
          .withTimeout(2.0)
      )
      .andThen(WaitCommand(0.10))
      .andThen(robot.intake.outtakeCoralPivot())
      .andThen(
        WaitUntilCommand { !robot.intake.coralDetected() }
          .onlyIf { RobotBase.isReal() }
      )
      .andThen(WaitCommand(0.050))
      .andThen(robot.intake.stop())
  }

  fun ScoreL2PivotSide(robot: Robot, reefSide: FieldConstants.ReefSide): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.L2_PIVOT)
      .alongWith(
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(reefSide), translationSpeedLim = 1.0, translationAccelLim = 1.95)
          .andThen(PrintCommand("Actually reached auto tolerance!"))
          .withTimeout(2.0)
      )
      .andThen(WaitCommand(0.10))
      .andThen(robot.intake.outtakeCoralPivot())
      .andThen(
        WaitUntilCommand { !robot.intake.coralDetected() }
          .onlyIf { RobotBase.isReal() }
      )
      .andThen(WaitCommand(0.050))
      .andThen(robot.intake.stop())
  }

  fun ScoreL2PivotSideN(robot: Robot): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.L2_PIVOT)
      .andThen(WaitCommand(0.10))
      .andThen(robot.intake.outtakeCoralPivot())
      .andThen(
        WaitUntilCommand { !robot.intake.coralDetected() }
          .onlyIf { RobotBase.isReal() }
      )
      .andThen(WaitCommand(0.050))
      .andThen(robot.intake.stop())
  }

  fun Intake(robot: Robot): Command {
    return InstantCommand(robot.drive::stop)
      .andThen(robot.intake.intakeCoral())
      .andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE))
      .andThen(
        WaitUntilCommand { robot.intake.coralDetected() }
          .onlyIf { RobotBase.isReal() }
      )
      .andThen(robot.intake.stop())
  }

  fun PremoveIntake(robot: Robot): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.GROUND_INTAKE)
      .alongWith(robot.intake.intakeCoral())
  }

  fun GroundIntake(robot: Robot): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.GROUND_INTAKE)
      .alongWith(robot.intake.intakeCoral())
      .andThen(
        WaitUntilCommand { robot.intake.coralDetected() }
          .onlyIf { RobotBase.isReal() }
      )
      .andThen(
        robot.intake.stop()
      )
  }
}
