package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.generated.TunerConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both RIO and CANivore buses.
 */
public class PhoenixOdometryThread extends Thread {
  private final Lock signalsLock =
      new ReentrantLock(); // Prevents conflicts when registering signals
  private BaseStatusSignal[] signals = new BaseStatusSignal[] {};
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<Queue<Double>> genericQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private static PhoenixOdometryThread instance = null;

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  private PhoenixOdometryThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
  }

  @Override
  public void start() {
    if (timestampQueues.size() > 0) {
      super.start();
    }
  }

  /** Creates a new timestamp queue to be filled by the odometry thread. */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** Registers a Phoenix signal to be read at high frequency. Returns the output queue. */
  public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    Drive.odometryLock.lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[signals.length + 1];
      System.arraycopy(signals, 0, newSignals, 0, signals.length);
      newSignals[signals.length] = signal;
      signals = newSignals;
      genericSignals.add(() -> signal.getValueAsDouble());
      genericQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
      signalsLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    while (true) {
      // Wait for updates from all signals
      signalsLock.lock();
      try {
        if (TunerConstants.kCANBus.isNetworkFD()) {
          BaseStatusSignal.waitForAll(2.0, signals);
        } else {
          // waitForAll does not support blocking on multiple signals with a bus that is not CAN FD
          Thread.sleep((long) (1000.0 / Drive.ODOMETRY_FREQUENCY));
          if (signals.length > 0) BaseStatusSignal.refreshAll(signals);
        }
      } catch (InterruptedException e) {
        e.printStackTrace();
      } finally {
        signalsLock.unlock();
      }

      // Save to queues
      Drive.odometryLock.lock();
      try {
        // Calculate timestamp: FPGA time minus average CAN latency
        double timestamp = RobotController.getFPGATime() / 1.0e6;
        double totalLatency = 0.0;
        for (BaseStatusSignal signal : signals) {
          totalLatency += signal.getTimestamp().getLatency();
        }
        if (signals.length > 0) {
          timestamp -= totalLatency / signals.length;
        }

        // Add timestamp to all timestamp queues
        for (Queue<Double> queue : timestampQueues) {
          queue.offer(timestamp);
        }

        // Add signal values to their respective queues
        for (int i = 0; i < genericSignals.size(); i++) {
          genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
        }
      } finally {
        Drive.odometryLock.unlock();
      }
    }
  }
}
