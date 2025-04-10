package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import choreo.trajectory.SwerveSample
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.*
import frc.team449.Robot
import frc.team449.commands.driveAlign.SimpleReefAlign
import frc.team449.subsystems.FieldConstants
import frc.team449.subsystems.superstructure.SuperstructureGoal
import java.util.Optional

open class Routines(
  val robot: Robot
) {

  private val autoFactory = AutoFactory(
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
    val middleRoutine = autoFactory.newRoutine("prev/middle Test")
    val test = middleRoutine.trajectory("prev/middle test")

    middleRoutine.active().onTrue(
      Commands.sequence(
        test.resetOdometry(),
        test.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE))
      )
    )

    test.done().onTrue(
      Commands.sequence(
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.LEFT), translationSpeedLim = 2.0, translationAccelLim = 1.4).alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
        ),
        robot.drive.driveStop(),
        WaitCommand(0.15),
        robot.intake.outtakeCoral().andThen(WaitUntilCommand { !robot.intake.coralDetected() }),
        WaitCommand(0.15).onlyIf { RobotBase.isReal() },
        robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
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
  //pass in "l" or "r" for direction
  private fun ground3HalfL4(direction: String): AutoRoutine {
    val routine = autoFactory.newRoutine("3.5 Ground L4 ${if (direction == "r") "Right" else "Left"}")
    val preloadScore = routine.trajectory("GroundThreeHalf/1$direction")
    val firstPickup = routine.trajectory("GroundThreeHalf/2$direction")
    val firstPresagedScore = routine.trajectory("GroundThreeHalf/3$direction")
    val secondPickup = routine.trajectory("GroundThreeHalf/4$direction")
    val secondPresagedScore = routine.trajectory("GroundThreeHalf/5$direction")
    val thirdPickup = routine.trajectory("GroundThreeHalf/6$direction")

    routine.active().onTrue(
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
        scoreL4PivotSideDirectional(if(direction == "r") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        firstPickup.cmd().alongWith(intake()),
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
        scoreL4PivotSideDirectional(if(direction == "l") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        secondPickup.cmd().alongWith(intake()),
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
        scoreL4PivotSideDirectional(if(direction == "r") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        intake().alongWith(
          thirdPickup.cmd()
        )
      )
    )

    thirdPickup.done().onTrue(
      robot.drive.driveStop()
    )

    return routine
  }

  // three l4 starting from a side then the back two reefs then half
  fun rightGround3L4Half(): AutoRoutine {
    return ground3HalfL4("r")
  }

  fun leftGround3L4Half(): AutoRoutine {
    return ground3HalfL4("l")
  }

  // back l4 and then sides 2 l4
  private fun threeL4(direction: String): AutoRoutine {
    val middlesides = autoFactory.newRoutine("3 l4")
    val preloadScore = middlesides.trajectory("middleSides/1$direction")
    val firstPickup = middlesides.trajectory("middleSides/2$direction")
    val firstPresagedScore = middlesides.trajectory("middleSides/3$direction")
    val secondPickup = middlesides.trajectory("middleSides/4$direction")
    val secondPresagedScore = middlesides.trajectory("middleSides/5$direction")
    val end = middlesides.trajectory("middleSides/end$direction")

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
        scoreL4PivotSideDirectional(if(direction=="l") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        firstPickup.cmd().alongWith(intake()),
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
        scoreL4PivotSideDirectional(if(direction=="r") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        secondPickup.cmd().alongWith(intake()),
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
        scoreL4PivotSideDirectional(if(direction=="l") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        end.cmd().alongWith( robot.superstructureManager.requestGoal(SuperstructureGoal.STOW))
      )
    )

    return middlesides
  }

  fun left3L4(): AutoRoutine {
    return threeL4("l")
  }

  fun right3L4(): AutoRoutine {
    return threeL4("r")
  }

  private fun groundBack2L4L2(direction: String): AutoRoutine {
    val rightBack2l4l2 = autoFactory.newRoutine("2 l4 and l2")
    val scorePreloadB = rightBack2l4l2.trajectory("TwoL4L2/1$direction")
    val pickupMiddle = rightBack2l4l2.trajectory("TwoL4L2/2$direction")
    val scoreMiddleA = rightBack2l4l2.trajectory("TwoL4L2/3$direction")
    val pickupLeft = rightBack2l4l2.trajectory("TwoL4L2/4$direction")
    val scoreRightB = rightBack2l4l2.trajectory("TwoL4L2/5$direction")
    val pickupRight = rightBack2l4l2.trajectory("TwoL4L2/6$direction")
    val scoreLeftA = rightBack2l4l2.trajectory("TwoL4L2/7$direction")
    val end = rightBack2l4l2.trajectory("TwoL4L2/end$direction")

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
        scoreL4PivotSideDirectional(FieldConstants.ReefSide.LEFT),
        pickupMiddle.cmd().alongWith(intake()),
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
        scoreL4PivotSideDirectional(FieldConstants.ReefSide.RIGHT),
        pickupLeft.cmd().alongWith(intake()),
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
          scoreL2PivotDirectional(FieldConstants.ReefSide.RIGHT),
          pickupRight.cmd().alongWith(intake()),
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
          scoreL2PivotDirectional(FieldConstants.ReefSide.LEFT),
          end.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)),
          robot.drive.driveStop(),

          )
      )

    return rightBack2l4l2
  }

  fun rightGroundBack2L4L2(): AutoRoutine {
    return groundBack2L4L2("r")
  }

  fun leftGroundBack2L4L2(): AutoRoutine {
    return groundBack2L4L2("l")
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
        scoreL4PivotSide(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
        pickupMiddle.cmd().alongWith(intake()),
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
        scoreL4PivotSide(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
        pickupLeft.cmd().alongWith(intake()),
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
          scoreL2PivotSide(),
          pickupRight.cmd().alongWith(intake()),
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
          scoreL2PivotSide(),
          end.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)),
          robot.drive.driveStop(),

          )
      )

    return leftBack2l4l2
  }



  fun americanRoutineOptimal(): AutoRoutine {
    val optimalAmerican = autoFactory.newRoutine("opt Ameriacn")
    val l4ATraj = optimalAmerican.trajectory("GroundThreeHalf/L4A (I)")
    val l4BTraj = optimalAmerican.trajectory("GroundThreeHalf/l4B")
    val loli1Traj = optimalAmerican.trajectory("GroundThreeHalf/Loli 1")
    val loli2Traj = optimalAmerican.trajectory("GroundThreeHalf/Loli 2")
    val l3Traj = optimalAmerican.trajectory("GroundThreeHalf/l3B")
    val loli3Traj = optimalAmerican.trajectory("GroundThreeHalf/Loli 3")

    optimalAmerican.active().onTrue(
      l4ATraj.resetOdometry().andThen(l4ATraj.cmd())

    )

    l4ATraj.done().onTrue(
      Commands.sequence(
        scoreL4PivotSideDirectional(FieldConstants.ReefSide.LEFT),
        loli1Traj.cmd().alongWith(intake()),
        l4BTraj.cmd(),
        scoreL4PivotSideDirectional(FieldConstants.ReefSide.RIGHT),
        intake().alongWith(loli2Traj.cmd()),
        l3Traj.cmd().andThen(scoreL2PivotDirectional(FieldConstants.ReefSide.RIGHT)),
        loli3Traj.cmd().alongWith(intake())

      )
    )

    return optimalAmerican
  }

  // Elevator is cooked!
  // autoChooser that will be displayed on dashboard
  fun addOptions(autoChooser: AutoChooser) {
    autoChooser.addRoutine("right 3.5 L4", this::rightGround3L4Half)
    autoChooser.addRoutine("left 3.5 fL4", this::leftGround3L4Half)

    autoChooser.addRoutine("RightBackL4+L2", this::rightGroundBack2L4L2)
    autoChooser.addRoutine("LeftBackL4+L2", this::leftGroundBack2L4L2)

    autoChooser.addRoutine("Left Middle&Sides", this::left3L4)
    autoChooser.addRoutine("Right Middle&Sides", this::right3L4)

    autoChooser.addRoutine("Taxi", this::taxi)
    autoChooser.addRoutine("No align LeftBackL4+L2", this::noAlignLeftBack2L4l2)
    autoChooser.addRoutine("optimal stuff", this::americanRoutineOptimal)
    autoChooser.addRoutine("middle routine", this::middleRoutine)
  }

  private fun scoreL4PivotSideDirectional(reefSide: FieldConstants.ReefSide): Command {
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


  private fun scoreL4PivotSide(): Command {
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

  private fun scoreL3PivotSideDirectional(reefSide: FieldConstants.ReefSide): Command {
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

  private fun scoreL2PivotDirectional(reefSide: FieldConstants.ReefSide): Command {
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

  private fun scoreL2PivotSide(): Command {
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

  private fun premoveIntake(): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.GROUND_INTAKE)
      .alongWith(robot.intake.intakeCoral())
  }

  private fun intake(): Command {
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
