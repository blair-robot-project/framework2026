package frc.team449.subsystems.drive

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.StatusSignal
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.RobotController
import frc.team449.generated.TunerConstants
import java.util.Queue
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.locks.Lock
import java.util.concurrent.locks.ReentrantLock
import java.util.function.DoubleSupplier

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 * * * This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
object PhoenixOdometryThread : Thread() {
  private val signalsLock: Lock = ReentrantLock() // Prevents conflicts when registering signals
  private var phoenixSignals: Array<BaseStatusSignal> = arrayOf()
  private val genericSignals: MutableList<DoubleSupplier> = ArrayList()
  private val phoenixQueues: MutableList<Queue<Double>> = ArrayList()
  private val genericQueues: MutableList<Queue<Double>> = ArrayList()
  private val timestampQueues: MutableList<Queue<Double>> = ArrayList()

  private val isCANFD = CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD

  init {
    this.setName("PhoenixOdometryThread")
    this.setDaemon(true)
  }

  override fun start() {
    if (timestampQueues.isNotEmpty()) {
      super.start()
    }
  }

  /** Registers a Phoenix signal to be read from the thread.  */
  fun registerSignal(signal: StatusSignal<Angle>): Queue<Double> {
    val queue: Queue<Double> = ArrayBlockingQueue(20)
    signalsLock.lock()
    SwerveDrive.odometryLock.lock()

    try {
      val newSignals: Array<BaseStatusSignal> = arrayOf()
      System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.size)
      newSignals[phoenixSignals.size] = signal
      phoenixSignals = newSignals
      phoenixQueues.add(queue)
    } finally {
      signalsLock.unlock()
      SwerveDrive.odometryLock.unlock()
    }

    return queue
  }

  /** Registers a generic signal to be read from the thread.  */
  fun registerSignal(signal: DoubleSupplier): Queue<Double> {
    val queue: Queue<Double> = ArrayBlockingQueue(20)
    signalsLock.lock()
    SwerveDrive.odometryLock.lock()

    try {
      genericSignals.add(signal)
      genericQueues.add(queue)
    } finally {
      signalsLock.unlock()
      SwerveDrive.odometryLock.unlock()
    }

    return queue
  }

  /** Returns a new queue that returns timestamp values for each sample.  */
  fun makeTimestampQueue(): Queue<Double> {
    val queue: Queue<Double> = ArrayBlockingQueue(20)
    SwerveDrive.odometryLock.lock()

    try {
      timestampQueues.add(queue)
    } finally {
      SwerveDrive.odometryLock.unlock()
    }

    return queue
  }

  override fun run() {
    while (true) {
      // wait for updates from all signals
      signalsLock.lock()
      try {
        if (isCANFD && phoenixSignals.isNotEmpty()) {
          BaseStatusSignal.waitForAll(2.0 / SwerveDrive.ODOMETRY_FREQUENCY, *phoenixSignals)
        } else {
          sleep((1000.0 / SwerveDrive.ODOMETRY_FREQUENCY).toLong())
          if (phoenixSignals.isNotEmpty()) {
            BaseStatusSignal.refreshAll(*phoenixSignals)
          }
        }
      } catch (e: InterruptedException) {
        e.printStackTrace()
      } finally {
        signalsLock.unlock()
      }

      // save new data to queues
      SwerveDrive.odometryLock.lock()
      try {
        var timestamp = RobotController.getFPGATime() / 1e6
        var totalLatency = 0.0
        for (signal in phoenixSignals) {
          totalLatency += signal.timestamp.latency
        }
        if (phoenixSignals.isNotEmpty()) {
          timestamp -= totalLatency / phoenixSignals.size
        }

        // Add new samples to queues
        for (i in phoenixSignals.indices) {
          phoenixQueues[i].offer(phoenixSignals[i].valueAsDouble)
        }
        for (i in genericSignals.indices) {
          genericQueues[i].offer(genericSignals[i].asDouble)
        }
        for (i in timestampQueues.indices) {
          timestampQueues[i].offer(timestamp)
        }
      } finally {
        SwerveDrive.odometryLock.unlock()
      }
    }
  }
}
