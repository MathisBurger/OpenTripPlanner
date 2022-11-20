package org.opentripplanner.transit.raptor.heuristic;

import static org.opentripplanner.util.lang.IntUtils.intArray;

import java.util.Arrays;
import java.util.BitSet;
import java.util.Iterator;
import org.opentripplanner.transit.raptor.api.transit.CostCalculator;
import org.opentripplanner.transit.raptor.api.transit.IntIterator;
import org.opentripplanner.transit.raptor.api.transit.RaptorAccessEgress;
import org.opentripplanner.transit.raptor.api.transit.RaptorTimeTable;
import org.opentripplanner.transit.raptor.api.transit.RaptorTransfer;
import org.opentripplanner.transit.raptor.api.transit.RaptorTransferConstraint;
import org.opentripplanner.transit.raptor.api.transit.RaptorTripSchedule;
import org.opentripplanner.transit.raptor.rangeraptor.internalapi.Heuristics;
import org.opentripplanner.transit.raptor.rangeraptor.internalapi.RoundProvider;
import org.opentripplanner.transit.raptor.rangeraptor.internalapi.WorkerLifeCycle;
import org.opentripplanner.transit.raptor.util.BitSetIterator;

public class DefaultHeuristicRoutingStrategy<T extends RaptorTripSchedule>
  implements HeuristicRoutingStrategy<T> {

  public static final int UNREACHED = 999_999_999;
  public static final byte UNREACHED_ROUNDS = 99;
  private final int[] egressStops;
  private final byte[] bestNumOfTransfers;

  /** The best times to reach a stop, across rounds and iterations. */
  private final int[] times;

  /**
   * The best "on-board" arrival times to reach a stop, across rounds and iterations. It includes
   * both transit arrivals and access-on-board arrivals.
   */
  private final int[] transitArrivalTimes;

  private final int[] costs;

  private final int[] transitArrivalCosts;

  private final BitSet reachedByTransitCurrentRound;
  private final byte[] heuristicRounds;
  private int roundMaxLimit;

  /** Stops touched in the CURRENT round. */
  private BitSet reachedCurrentRound;
  /** Stops touched by in LAST round. */
  private BitSet reachedLastRound;
  private final CostCalculator<T> costCalculator;
  private final RoundProvider roundProvider;
  private T currentTrip;
  private int currentBoardingTime;
  private int currentBoardCost;
  private int currentBoardingTotalDuration;

  public DefaultHeuristicRoutingStrategy(
    int nStops,
    RoundProvider roundProvider,
    WorkerLifeCycle lifeCycle,
    CostCalculator<T> costCalculator,
    int[] egressStops,
    Heuristics previousHeuristic
  ) {
    this.bestNumOfTransfers = new byte[nStops];
    Arrays.fill(bestNumOfTransfers, UNREACHED_ROUNDS);
    this.times = intArray(nStops, UNREACHED);
    this.transitArrivalTimes = intArray(nStops, UNREACHED);
    this.costs = intArray(nStops, UNREACHED);
    this.transitArrivalCosts = intArray(nStops, UNREACHED);
    this.reachedByTransitCurrentRound = new BitSet(nStops);
    this.reachedCurrentRound = new BitSet(nStops);
    this.reachedLastRound = new BitSet(nStops);
    this.costCalculator = costCalculator;
    this.roundProvider = roundProvider;
    // TODO: Read in max rounds from config
    this.roundMaxLimit = 12;
    this.heuristicRounds = previousHeuristic.bestNumOfTransfersToByteArray((byte) roundMaxLimit);

    this.egressStops = egressStops;

    // Attach to Worker life cycle
    lifeCycle.onSetupIteration(ignore -> setupIteration());
    lifeCycle.onPrepareForNextRound(round -> prepareForNextRound());
    lifeCycle.onRoundComplete(this::roundComplete);
  }

  @Override
  public boolean isNewRoundAvailable() {
    return !reachedCurrentRound.isEmpty();
  }

  @Override
  public boolean isDestinationReachedInCurrentRound() {
    // This is fast enough, we could use a BitSet for egressStops, but it takes up more
    // memory and the performance is the same.
    for (final int egressStop : egressStops) {
      if (reachedByTransitCurrentRound.get(egressStop)) {
        return true;
      }
    }
    return false;
  }

  @Override
  public IntIterator stopsTouchedPreviousRound() {
    return new BitSetIterator(reachedLastRound);
  }

  @Override
  public IntIterator stopsTouchedByTransitCurrentRound() {
    return new BitSetIterator(reachedByTransitCurrentRound);
  }

  @Override
  public boolean isStopReachedInPreviousRound(int stopIndex) {
    return reachedLastRound.get(stopIndex);
  }

  @Override
  public int bestTimePreviousRound(int stopIndex) {
    return times[stopIndex];
  }

  public int bestCostPreviousRound(int stopIndex) {
    return costs[stopIndex];
  }

  @Override
  public void prepareForTransitWith(RaptorTimeTable<T> timetable) {
    this.currentTrip = timetable.getHeuristicTrip();
    this.currentBoardingTime = UNREACHED;
    this.currentBoardingTotalDuration = UNREACHED;
    this.currentBoardCost = UNREACHED;
  }

  @Override
  public void board(int stopIndex, int stopPos, int boardSlack, boolean hasConstrainedTransfer) {
    final int previousDuration = bestTimePreviousRound(stopIndex);
    final int boardingTime = currentTrip.arrival(stopPos);
    final int slack = hasConstrainedTransfer ? 0 : boardSlack;

    if (
      currentBoardingTotalDuration == UNREACHED ||
      (
        currentBoardingTotalDuration >
        (previousDuration + currentBoardingTime - boardingTime + slack)
      )
    ) {
      currentBoardingTime = boardingTime;
      currentBoardingTotalDuration = previousDuration + slack;

      // TODO: check what types of constrained transfers are available
      final int boardCost = hasConstrainedTransfer
        ? 0
        : costCalculator.boardingCost(
          roundProvider.round() < 2,
          previousDuration,
          stopIndex,
          previousDuration + slack,
          currentTrip,
          // TODO add best constraint to heuristic trip
          RaptorTransferConstraint.REGULAR_TRANSFER
        );

      //  TODO, we should add cost + time to an array, so we can check both what is the minimum time
      //  and cost separately at arrival
      currentBoardCost = bestCostPreviousRound(stopIndex) + boardCost;
    }
  }

  @Override
  public void alight(int stopIndex, int stopPos, int alightSlack) {
    if (currentBoardingTime != UNREACHED) {
      final int transitDuration = currentBoardingTime - currentTrip.departure(stopPos);
      final int transitCost = costCalculator.transitArrivalCost(
        currentBoardCost,
        alightSlack,
        transitDuration,
        currentTrip,
        stopIndex
      );

      updateNewBestTimeCostAndRound(
        stopIndex,
        currentBoardingTotalDuration + alightSlack + transitDuration,
        currentBoardCost + transitCost,
        true
      );
    }
  }

  @Override
  public void transferToStops(int fromStop, Iterator<? extends RaptorTransfer> transfers) {
    final int prevDuration = bestTimePreviousRound(fromStop);
    final int prevCost = bestCostPreviousRound(fromStop);

    while (transfers.hasNext()) {
      RaptorTransfer it = transfers.next();
      updateNewBestTimeCostAndRound(
        it.stop(),
        it.durationInSeconds() + prevDuration,
        it.generalizedCost() + prevCost,
        false
      );
    }
  }

  @Override
  public void setAccessToStop(RaptorAccessEgress accessPath) {
    updateNewBestTimeCostAndRound(
      accessPath.stop(),
      accessPath.durationInSeconds(),
      accessPath.generalizedCost(),
      accessPath.stopReachedOnBoard()
    );
  }

  public Heuristics heuristics() {
    return new ReverseHeuristicsAdapter(bestNumOfTransfers, times, costs, egressStops);
  }

  /**
   * Clear all reached flags before we start a new iteration. This is important so stops visited in
   * the previous iteration in the last round does not "overflow" into the next iteration.
   */
  private void setupIteration() {
    // clear all touched stops to avoid constant reëxploration
    reachedCurrentRound.clear();
    reachedByTransitCurrentRound.clear();
  }

  private void swapReachedCurrentAndLastRound() {
    BitSet tmp = reachedLastRound;
    reachedLastRound = reachedCurrentRound;
    reachedCurrentRound = tmp;
  }

  private void updateNewBestTimeCostAndRound(int stop, int time, int cost, boolean isTransit) {
    if (heuristicRounds[stop] + roundProvider.round() > roundMaxLimit) {
      return;
    }
    if (!isTransit || updateBestTransitArrivalTime(stop, time)) {
      updateBestTime(stop, time);
    }
    if (!isTransit || updateBestTransitArrivalCost(stop, cost)) {
      updateBestCost(stop, cost);
    }
    updateBestRound(stop);
  }

  private boolean updateBestTime(int stop, int time) {
    if (time < times[stop]) {
      times[stop] = time;
      reachedCurrentRound.set(stop);
      return true;
    }
    return false;
  }

  private boolean updateBestTransitArrivalTime(int stop, int time) {
    if (time < transitArrivalTimes[stop]) {
      transitArrivalTimes[stop] = time;
      reachedByTransitCurrentRound.set(stop);
      return true;
    }
    return false;
  }

  private boolean updateBestCost(int stop, int cost) {
    if (cost < costs[stop]) {
      costs[stop] = cost;
      reachedCurrentRound.set(stop);
      return true;
    }
    return false;
  }

  private boolean updateBestTransitArrivalCost(int stop, int cost) {
    if (cost < transitArrivalCosts[stop]) {
      transitArrivalCosts[stop] = cost;
      reachedByTransitCurrentRound.set(stop);
      return true;
    }
    return false;
  }

  void updateBestRound(int stop) {
    final byte numOfTransfers = (byte) (roundProvider.round() - 1);
    if (numOfTransfers < bestNumOfTransfers[stop]) {
      bestNumOfTransfers[stop] = numOfTransfers;
    }
  }

  /**
   * Prepare this class for the next round updating reached flags.
   */
  private void prepareForNextRound() {
    swapReachedCurrentAndLastRound();
    reachedCurrentRound.clear();
    reachedByTransitCurrentRound.clear();
  }

  private void roundComplete(boolean complete) {
    if (complete) {
      // TODO read param form config
      roundMaxLimit = Math.min(roundMaxLimit, roundProvider.round() + 5 + 1);
    }
  }
}
