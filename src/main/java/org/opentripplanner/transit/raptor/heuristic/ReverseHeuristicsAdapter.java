package org.opentripplanner.transit.raptor.heuristic;

import java.util.Arrays;
import java.util.function.IntUnaryOperator;
import org.opentripplanner.transit.raptor.rangeraptor.internalapi.HeuristicAtStop;
import org.opentripplanner.transit.raptor.rangeraptor.internalapi.Heuristics;
import org.opentripplanner.util.lang.IntUtils;

public class ReverseHeuristicsAdapter implements Heuristics {

  private final byte[] bestNumOfTransfers;
  private final int[] times;
  private final int[] costs;
  private final int[] egressStops;

  public ReverseHeuristicsAdapter(
    byte[] bestNumOfTransfers,
    int[] times,
    int[] costs,
    int[] egressStops
  ) {
    this.bestNumOfTransfers = bestNumOfTransfers;
    this.times = times;
    this.costs = costs;
    this.egressStops = egressStops;
  }

  @Override
  public int[] bestTravelDurationToIntArray(int unreached) {
    return toIntArray(size(), unreached, this::bestTravelDuration);
  }

  @Override
  public byte[] bestNumOfTransfersToByteArray(byte unreached) {
    return toByteArray(size(), unreached, this::bestNumOfTransfers);
  }

  @Override
  public int[] bestGeneralizedCostToIntArray(int unreached) {
    return toIntArray(size(), unreached, this::bestGeneralizedCost);
  }

  @Override
  public HeuristicAtStop createHeuristicAtStop(int stop) {
    return reached(stop)
      ? new HeuristicAtStop(times[stop], bestNumOfTransfers[stop], costs[stop])
      : HeuristicAtStop.UNREACHED;
  }

  @Override
  public int size() {
    return bestNumOfTransfers.length;
  }

  @Override
  public int bestOverallJourneyTravelDuration() {
    int best = DefaultHeuristicRoutingStrategy.UNREACHED;
    for (int stop : egressStops) {
      if (times[stop] < best) {
        best = times[stop];
      }
    }
    return best;
  }

  @Override
  public int bestOverallJourneyNumOfTransfers() {
    int best = DefaultHeuristicRoutingStrategy.UNREACHED;
    for (int stop : egressStops) {
      if (bestNumOfTransfers[stop] < best) {
        best = bestNumOfTransfers[stop];
      }
    }
    return best;
  }

  @Override
  public int minWaitTimeForJourneysReachingDestination() {
    return 0;
  }

  @Override
  public boolean destinationReached() {
    for (int stop : egressStops) {
      if (reached(stop)) {
        return true;
      }
    }
    return false;
  }

  private boolean reached(int stop) {
    return bestNumOfTransfers[stop] < DefaultHeuristicRoutingStrategy.UNREACHED_ROUNDS;
  }

  private int bestTravelDuration(int stop) {
    return times[stop];
  }

  private byte bestNumOfTransfers(int stop) {
    return bestNumOfTransfers[stop];
  }

  private int bestGeneralizedCost(int stop) {
    return costs[stop];
  }

  /**
   * Convert one of heuristics to an int array.
   */
  private int[] toIntArray(int size, int unreached, IntUnaryOperator supplier) {
    int[] a = IntUtils.intArray(size, unreached);
    for (int i = 0; i < a.length; i++) {
      if (reached(i)) {
        a[i] = supplier.applyAsInt(i);
      }
    }
    return a;
  }

  /**
   * Convert one of heuristics to an int array.
   */
  private byte[] toByteArray(int size, byte unreached, IntToByteOperator supplier) {
    byte[] a = new byte[size];
    Arrays.fill(a, unreached);
    for (int i = 0; i < a.length; i++) {
      if (reached(i)) {
        a[i] = supplier.apply(i);
      }
    }
    return a;
  }

  @FunctionalInterface
  public interface IntToByteOperator {
    byte apply(int operand);
  }
}
