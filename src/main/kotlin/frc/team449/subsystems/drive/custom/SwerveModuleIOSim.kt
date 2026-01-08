package frc.team449.subsystems.drive.custom

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import frc.team449.generated.TunerConstants
import frc.team449.util.PhoenixUtil
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation
import org.ironmaple.simulation.motorsims.SimulatedMotorController
import java.util.Arrays
import kotlin.math.abs
import kotlin.math.sign

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
class SwerveModuleIOSim(
    val moduleSimulation: SwerveModuleSimulation
) : SwerveModuleIO {

    private val driveMotor: SimulatedMotorController.GenericMotorController =
        moduleSimulation
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(Units.Amps.of(TunerConstants.FrontLeft.SlipCurrent))

    // TODO: find home for steer current limit
    private val turnMotor: SimulatedMotorController.GenericMotorController =
        moduleSimulation
            .useGenericControllerForSteer()
            .withCurrentLimit(Units.Amps.of(20.0))

    private var driveClosedLoop = false
    private var turnClosedLoop = false
    private val driveController = PIDController(DRIVE_KP, 0.0, DRIVE_KD)
    private val turnController = PIDController(TURN_KP, 0.0, TURN_KD)

    private var driveFFVolts = 0.0
    private var driveAppliedVolts = 0.0
    private var turnAppliedVolts = 0.0

    init {
        // enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI)
    }

    override fun updateInputs(inputs: SwerveModuleIO.SwerveModuleIOInputs) {
        // run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts =
                driveFFVolts + driveController.calculate(
                moduleSimulation.driveWheelFinalSpeed.`in`(Units.RadiansPerSecond)
            )
        } else {
            driveController.reset()
        }
        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(
                moduleSimulation.steerAbsoluteFacing.radians
            )
        } else {
            turnController.reset()
        }

        driveMotor.requestVoltage(Units.Volts.of(driveAppliedVolts))
        turnMotor.requestVoltage(Units.Volts.of(turnAppliedVolts))

        // update drive inputs
        inputs.driveConnected = true
        inputs.drivePositionRad = moduleSimulation.driveWheelFinalPosition.`in`(Units.Radians)
        inputs.driveVelocityRadPerSec =
            moduleSimulation.driveWheelFinalSpeed.`in`(Units.RadiansPerSecond)
        inputs.driveAppliedVolts = driveAppliedVolts
        inputs.driveCurrentAmps = abs(moduleSimulation.driveMotorStatorCurrent.`in`(Units.Amps))

        // update turn inputs
        inputs.turnConnected = true
        inputs.turnEncoderConnected = true
        inputs.turnAbsolutePosition = moduleSimulation.steerAbsoluteFacing
        inputs.turnPosition = moduleSimulation.steerAbsoluteFacing
        inputs.turnVelocityRadPerSec =
            moduleSimulation.steerAbsoluteEncoderSpeed.`in`(Units.RadiansPerSecond)
        inputs.turnAppliedVolts = turnAppliedVolts
        inputs.turnCurrentAmps = abs(moduleSimulation.steerMotorStatorCurrent.`in`(Units.Amps))

        // update odometry inputs (50 Hz because high-frequency odometry in sim doesn't matter)
        inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps()
        inputs.odometryDrivePositionsRad = Arrays.stream(moduleSimulation.cachedDriveWheelFinalPositions)
            .mapToDouble { angle: Angle -> angle.`in`(Units.Radians) }
            .toArray()
        inputs.odometryTurnPositions = moduleSimulation.cachedSteerAbsolutePositions
    }

    override fun setDriveOpenLoop(output: Double) {
        driveClosedLoop = false
        driveAppliedVolts = output
    }

    override fun setTurnOpenLoop(output: Double) {
        turnClosedLoop = false
        turnAppliedVolts = output
    }

    override fun setDriveVelocity(velocityRadPerSec: Double) {
        driveClosedLoop = true
        driveFFVolts = DRIVE_KS * sign(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec
        driveController.setSetpoint(velocityRadPerSec)
    }

    override fun setTurnPosition(rotation: Rotation2d) {
        turnClosedLoop = true
        turnController.setSetpoint(rotation.radians)
    }

    companion object {
        // TunerConstants doesn't support separate sim constants, so they are declared locally
        private const val DRIVE_KP = 0.05
        private const val DRIVE_KD = 0.0
        private const val DRIVE_KS = 0.0
        private const val DRIVE_KV_ROT = 0.91035 // same units as TunerConstants: (volt * secs) / rotation
        private val DRIVE_KV = 1.0 / edu.wpi.first.math.util.Units.rotationsToRadians(1.0 / DRIVE_KV_ROT)

        private const val TURN_KP = 8.0
        private const val TURN_KD = 0.0

        private val DRIVE_GEARBOX: DCMotor = DCMotor.getKrakenX60Foc(1)
        private val TURN_GEARBOX: DCMotor = DCMotor.getKrakenX60Foc(1)
    }
}
