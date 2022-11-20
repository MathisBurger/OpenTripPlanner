package org.opentripplanner.transit.raptor.rangeraptor.multicriteria.heuristic;

import org.opentripplanner.transit.raptor.api.transit.RaptorTripSchedule;
import org.opentripplanner.transit.raptor.rangeraptor.debug.DebugHandlerFactory;
import org.opentripplanner.transit.raptor.rangeraptor.internalapi.HeuristicAtStop;
import org.opentripplanner.transit.raptor.rangeraptor.internalapi.Heuristics;
import org.opentripplanner.transit.raptor.rangeraptor.internalapi.RoundProvider;
import org.opentripplanner.transit.raptor.rangeraptor.multicriteria.arrivals.AbstractStopArrival;
import org.opentripplanner.transit.raptor.rangeraptor.path.DestinationArrivalPaths;
import org.opentripplanner.transit.raptor.rangeraptor.transit.RoundTracker;

/**
 * A wrapper around {@link Heuristics} to cache elements to avoid recalculation of heuristic
 * properties.
 *
 * @param <T> The TripSchedule type defined by the user of the raptor API.
 */
public final class HeuristicsProvider<T extends RaptorTripSchedule> {

  private final Heuristics heuristics;
  private final RoundTracker roundProvider;
  private final DestinationArrivalPaths<T> paths;
  private final HeuristicAtStop[] stops;
  private final DebugHandlerFactory<T> debugHandlerFactory;

  public HeuristicsProvider() {
    this(null, null, null, null);
  }

  public HeuristicsProvider(
    Heuristics heuristics,
    RoundProvider roundProvider,
    DestinationArrivalPaths<T> paths,
    DebugHandlerFactory<T> debugHandlerFactory
  ) {
    this.heuristics = heuristics;
    this.roundProvider = (RoundTracker) roundProvider;
    this.paths = paths;
    this.stops = heuristics == null ? null : new HeuristicAtStop[heuristics.size()];
    this.debugHandlerFactory = debugHandlerFactory;
  }

  /**
   * This is a very effective optimization, enabled by the {@link org.opentripplanner.transit.raptor.api.request.Optimization#PARETO_CHECK_AGAINST_DESTINATION}.
   */
  public boolean rejectDestinationArrivalBasedOnHeuristic(AbstractStopArrival<T> arrival) {
    if (heuristics == null || paths.isEmpty()) {
      return false;
    }
    boolean rejected = !qualify(
      arrival.stop(),
      arrival.arrivalTime(),
      arrival.travelDuration(),
      arrival.cost()
    );

    if (rejected) {
      debugRejectByOptimization(arrival);
    }
    return rejected;
  }

  /* private methods */

  private void debugRejectByOptimization(AbstractStopArrival<T> arrival) {
    if (debugHandlerFactory.isDebugStopArrival(arrival.stop())) {
      String details = rejectErrorMessage(arrival.stop()) + ", Existing paths: " + paths;

      debugHandlerFactory
        .debugStopArrival()
        .reject(
          arrival,
          null,
          "The element is rejected because the destination is not reachable within the limit " +
          "based on heuristic. Details: " +
          details
        );
    }
  }

  /**
   * This is used to make an optimistic guess for the best possible arrival at the destination,
   * using the given arrival and a pre-calculated heuristics.
   */
  private boolean qualify(int stop, int arrivalTime, int travelDuration, int cost) {
    HeuristicAtStop h = get(stop);

    if (h == HeuristicAtStop.UNREACHED) {
      return false;
    }
    int minArrivalTime = arrivalTime + h.minTravelDuration();
    int minNumberOfRounds = roundProvider.round() + h.minNumTransfers();
    if (minNumberOfRounds > roundProvider.roundMaxLimit()) {
      return false;
    }
    int minTravelDuration = travelDuration + h.minTravelDuration();
    int minCost = cost + h.minCost();
    int departureTime = minArrivalTime - minTravelDuration;
    return paths.qualify(departureTime, minArrivalTime, minNumberOfRounds - 1, minCost);
  }

  private String rejectErrorMessage(int stop) {
    return get(stop) == HeuristicAtStop.UNREACHED
      ? "The stop was not reached in the heuristic calculation."
      : get(stop).toString();
  }

  private HeuristicAtStop get(int stop) {
    if (stops[stop] == null) {
      stops[stop] = heuristics.createHeuristicAtStop(stop);
    }
    return stops[stop];
  }
}
