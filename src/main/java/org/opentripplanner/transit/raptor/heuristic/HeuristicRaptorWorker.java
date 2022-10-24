package org.opentripplanner.transit.raptor.heuristic;

import java.util.Collection;
import java.util.List;
import org.opentripplanner.transit.raptor.api.debug.RaptorTimers;
import org.opentripplanner.transit.raptor.api.path.Path;
import org.opentripplanner.transit.raptor.api.response.StopArrivals;
import org.opentripplanner.transit.raptor.api.transit.IntIterator;
import org.opentripplanner.transit.raptor.api.transit.RaptorAccessEgress;
import org.opentripplanner.transit.raptor.api.transit.RaptorTransitDataProvider;
import org.opentripplanner.transit.raptor.api.transit.RaptorTripSchedule;
import org.opentripplanner.transit.raptor.rangeraptor.internalapi.RoundProvider;
import org.opentripplanner.transit.raptor.rangeraptor.internalapi.SlackProvider;
import org.opentripplanner.transit.raptor.rangeraptor.internalapi.Worker;
import org.opentripplanner.transit.raptor.rangeraptor.lifecycle.LifeCycleEventPublisher;
import org.opentripplanner.transit.raptor.rangeraptor.transit.AccessPaths;
import org.opentripplanner.transit.raptor.rangeraptor.transit.RoundTracker;
import org.opentripplanner.transit.raptor.rangeraptor.transit.TransitCalculator;
import org.opentripplanner.transit.raptor.util.BitSetIterator;

public class HeuristicRaptorWorker<T extends RaptorTripSchedule> implements Worker<T> {

  private final HeuristicRoutingStrategy<T> transitWorker;

  /**
   * The round tracker keep track for the current Raptor round, and abort the search if the round
   * max limit is reached.
   */
  private final RoundTracker roundTracker;

  private final RaptorTransitDataProvider<T> transitData;

  private final SlackProvider slackProvider;

  private final TransitCalculator<T> calculator;

  private final RaptorTimers timers;

  private final AccessPaths accessPaths;

  private final LifeCycleEventPublisher lifeCycle;

  private final int minNumberOfRounds;

  private final boolean enableTransferConstraints;

  public HeuristicRaptorWorker(
    HeuristicRoutingStrategy<T> transitWorker,
    RaptorTransitDataProvider<T> transitData,
    SlackProvider slackProvider,
    AccessPaths accessPaths,
    RoundProvider roundProvider,
    TransitCalculator<T> calculator,
    LifeCycleEventPublisher lifeCyclePublisher,
    RaptorTimers timers,
    boolean enableTransferConstraints
  ) {
    this.transitWorker = transitWorker;
    this.transitData = transitData;
    this.slackProvider = slackProvider;
    this.calculator = calculator;
    this.timers = timers;
    this.accessPaths = accessPaths;
    this.minNumberOfRounds = accessPaths.calculateMaxNumberOfRides();
    this.enableTransferConstraints = enableTransferConstraints;

    // We do a cast here to avoid exposing the round tracker  and the life cycle publisher to
    // "everyone" by providing access to it in the context.
    this.roundTracker = (RoundTracker) roundProvider;
    this.lifeCycle = lifeCyclePublisher;
  }

  @Override
  public void route() {
    timers.route(this::routeInternal);
  }

  private void routeInternal() {
    lifeCycle.notifyRouteSearchStart(calculator.searchForward());
    transitData.setup();
    lifeCycle.setupIteration(0);
    findAccessOnStreetForRound();

    while (hasMoreRounds()) {
      lifeCycle.prepareForNextRound(roundTracker.nextRound());

      // NB since we have transfer limiting not bothering to cut off search when there are no
      // more transfers as that will be rare and complicates the code
      timers.findTransitForRound(this::findTransitForRound);

      findAccessOnBoardForRound();

      timers.findTransfersForRound(this::findTransfersForRound);

      lifeCycle.roundComplete(transitWorker.isDestinationReachedInCurrentRound());

      findAccessOnStreetForRound();
    }

    // This state is repeatedly modified as the outer loop progresses over departure minutes.
    // We have to be careful here, the next iteration will modify the state, so we need to make
    // protective copies of any information we want to retain.
    lifeCycle.iterationComplete();
  }

  @Override
  public Collection<Path<T>> paths() {
    return List.of();
  }

  @Override
  public StopArrivals stopArrivals() {
    return null;
  }

  /**
   * Check if the RangeRaptor should continue with a new round.
   */
  private boolean hasMoreRounds() {
    if (round() < minNumberOfRounds) {
      return true;
    }
    return transitWorker.isNewRoundAvailable() && roundTracker.hasMoreRounds();
  }

  private void findTransitForRound() {
    IntIterator stops = transitWorker.stopsTouchedPreviousRound();
    IntIterator routeIndexIterator = transitData.routeIndexIterator(stops);

    while (routeIndexIterator.hasNext()) {
      var routeIndex = routeIndexIterator.next();
      var route = transitData.getRouteForIndex(routeIndex);
      var pattern = route.pattern();
      var txSearch = enableTransferConstraints
        ? calculator.transferConstraintsSearch(transitData, routeIndex)
        : null;

      int alightSlack = slackProvider.alightSlack(pattern.slackIndex());
      int boardSlack = slackProvider.boardSlack(pattern.slackIndex());

      transitWorker.prepareForTransitWith(route.timetable());

      IntIterator stop = calculator.patternStopIterator(pattern.numberOfStopsInPattern());

      while (stop.hasNext()) {
        int stopPos = stop.next();
        int stopIndex = pattern.stopIndex(stopPos);

        // attempt to alight if we're on board, this is done above the board search
        // so that we don't alight on first stop boarded
        if (calculator.alightingPossibleAt(pattern, stopPos)) {
          transitWorker.alight(stopIndex, stopPos, alightSlack);
        }

        if (calculator.boardingPossibleAt(pattern, stopPos)) {
          if (transitWorker.isStopReachedInPreviousRound(stopIndex)) {
            boolean hasConstrainedTransfer =
              enableTransferConstraints && txSearch.transferExist(stopPos);

            transitWorker.board(stopIndex, stopPos, boardSlack, hasConstrainedTransfer);
          }
        }
      }
    }
    lifeCycle.transitsForRoundComplete();
  }

  private void findTransfersForRound() {
    IntIterator it = transitWorker.stopsTouchedByTransitCurrentRound();

    while (it.hasNext()) {
      final int fromStop = it.next();
      // no need to consider loop transfers, since we don't mark patterns here any more
      // loop transfers are already included by virtue of those stops having been reached
      transitWorker.transferToStops(fromStop, calculator.getTransfers(transitData, fromStop));
    }

    lifeCycle.transfersForRoundComplete();
  }

  private void findAccessOnStreetForRound() {
    addAccessPaths(accessPaths.arrivedOnStreetByNumOfRides().get(round()));
  }

  private void findAccessOnBoardForRound() {
    addAccessPaths(accessPaths.arrivedOnBoardByNumOfRides().get(round()));
  }

  /**
   * Set the departure time in the scheduled search to the given departure time, and prepare for the
   * scheduled search at the next-earlier minute.
   */
  private void addAccessPaths(Collection<RaptorAccessEgress> accessPaths) {
    if (accessPaths == null) {
      return;
    }

    for (RaptorAccessEgress it : accessPaths) {
      transitWorker.setAccessToStop(it);
    }
  }

  private int round() {
    return roundTracker.round();
  }
}
