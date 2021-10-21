package org.opentripplanner.routing.algorithm.raptor.transit.request;

import java.time.Instant;
import java.time.ZonedDateTime;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import org.opentripplanner.routing.algorithm.raptor.transit.RaptorTransferIndex;
import java.util.function.Function;
import org.opentripplanner.routing.algorithm.raptor.transit.TransitLayer;
import org.opentripplanner.routing.algorithm.raptor.transit.TripSchedule;
import org.opentripplanner.routing.algorithm.raptor.transit.cost.DefaultCostCalculator;
import org.opentripplanner.routing.algorithm.raptor.transit.cost.FactorStrategy;
import org.opentripplanner.routing.algorithm.raptor.transit.mappers.McCostParamsMapper;
import org.opentripplanner.routing.api.request.RoutingRequest;
import org.opentripplanner.transit.raptor.api.transit.CostCalculator;
import org.opentripplanner.transit.raptor.api.transit.IntIterator;
import org.opentripplanner.transit.raptor.api.transit.RaptorRoute;
import org.opentripplanner.transit.raptor.api.transit.RaptorTransfer;
import org.opentripplanner.transit.raptor.api.transit.RaptorTransitDataProvider;


/**
 * This is the data provider for the Range Raptor search engine. It uses data from the TransitLayer,
 * but filters it by dates and modes per request. Transfers durations are pre-calculated per request
 * based on walk speed.
 */
public class RaptorRoutingRequestTransitData implements RaptorTransitDataProvider<TripSchedule> {

  private final TransitLayer transitLayer;

  /**
   * Active trip patterns by stop index
   */
  private final List<List<TripPatternForDates>> activeTripPatternsPerStop;

  /**
   * Transfers by stop index
   */
  private final RaptorTransferIndex transfers;

  private final ZonedDateTime startOfTime;

  private final CostCalculator generalizedCostCalculator;


  public RaptorRoutingRequestTransitData(
      TransitLayer transitLayer,
      Instant departureTime,
      int additionalFutureSearchDays,
      TransitDataProviderFilter filter,
      RoutingRequest routingRequest
  ) {
    // Delegate to the creator to construct the needed data structures. The code is messy so
    // it is nice to NOT have it in the class. It isolate this code to only be available at
    // the time of construction
    RaptorRoutingRequestTransitDataCreator creator = new RaptorRoutingRequestTransitDataCreator(
        transitLayer,
        departureTime
    );

    this.transitLayer = transitLayer;
    this.startOfTime = creator.getSearchStartTime();
    this.activeTripPatternsPerStop = creator.createTripPatternsPerStop(
        additionalFutureSearchDays,
        filter
    );
    this.transfers = transitLayer.getRaptorTransfersForRequest(routingRequest);
    this.generalizedCostCalculator = new DefaultCostCalculator(
            McCostParamsMapper.map(routingRequest),
            transitLayer.getStopIndex().stopBoardAlightCosts
    );
  }

  @Override
  public Iterator<RaptorTransfer> getTransfersFromStop(int stopIndex) {
    return transfers.getForwardTransfers().get(stopIndex).iterator();
  }

  @Override
  public Iterator<? extends RaptorTransfer> getTransfersToStop(int stopIndex) {
    return transfers.getReversedTransfers().get(stopIndex).iterator();
  }

  @Override
  public Iterator<? extends RaptorRoute<TripSchedule>> routeIterator(IntIterator stops) {
    // A LinkedHashSet is used so that the iteration order is deterministic.
    Set<TripPatternForDates> activeTripPatternsForGivenStops = new LinkedHashSet<>();
    while (stops.hasNext()) {
      activeTripPatternsForGivenStops.addAll(activeTripPatternsPerStop.get(stops.next()));
    }
    return activeTripPatternsForGivenStops.iterator();
  }

  @Override
  public int numberOfStops() {
    return transitLayer.getStopCount();
  }

  @Override
  public CostCalculator multiCriteriaCostCalculator() {
    return generalizedCostCalculator;
  }

  public ZonedDateTime getStartOfTime() {
    return startOfTime;
  }


  /*--  HACK SØRLANDSBANEN  ::  BEGIN  --*/

  private RaptorRoutingRequestTransitData(
          RaptorRoutingRequestTransitData original,
          Function<FactorStrategy, FactorStrategy> mapFactors
  ) {
    this.transitLayer = original.transitLayer;
    this.startOfTime = original.startOfTime;
    this.activeTripPatternsPerStop = original.activeTripPatternsPerStop;
    this.transfers = original.transfers;
    this.generalizedCostCalculator = new DefaultCostCalculator(
            (DefaultCostCalculator) original.generalizedCostCalculator,
            mapFactors
    );
  }

  public RaptorTransitDataProvider<TripSchedule> enturHackSorlandsbanen(
          Function<FactorStrategy, FactorStrategy> mapFactors
  ) {
    return new RaptorRoutingRequestTransitData(this, mapFactors);
  }

  /*--  HACK SØRLANDSBANEN  ::  END  --*/
}
