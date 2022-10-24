package org.opentripplanner.transit.raptor.heuristic;

import java.util.Iterator;
import org.opentripplanner.transit.raptor.api.transit.IntIterator;
import org.opentripplanner.transit.raptor.api.transit.RaptorAccessEgress;
import org.opentripplanner.transit.raptor.api.transit.RaptorTimeTable;
import org.opentripplanner.transit.raptor.api.transit.RaptorTransfer;
import org.opentripplanner.transit.raptor.api.transit.RaptorTripSchedule;

public interface HeuristicRoutingStrategy<T extends RaptorTripSchedule> {
  boolean isNewRoundAvailable();
  boolean isDestinationReachedInCurrentRound();

  IntIterator stopsTouchedPreviousRound();
  IntIterator stopsTouchedByTransitCurrentRound();

  boolean isStopReachedInPreviousRound(int stopIndex);

  int bestTimePreviousRound(int stopIndex);

  void prepareForTransitWith(RaptorTimeTable<T> timetable);

  void board(int stopIndex, int stopPos, int boardSlack, boolean hasConstrainedTransfer);

  void alight(int stopIndex, int stopPos, int alightSlack);

  void transferToStops(int fromStop, Iterator<? extends RaptorTransfer> transfers);

  void setAccessToStop(RaptorAccessEgress it);
}
