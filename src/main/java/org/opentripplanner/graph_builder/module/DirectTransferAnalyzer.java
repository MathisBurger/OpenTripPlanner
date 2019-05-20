package org.opentripplanner.graph_builder.module;

import com.google.common.collect.Iterables;
import org.opentripplanner.graph_builder.annotation.TransferCouldNotBeRouted;
import org.opentripplanner.graph_builder.annotation.TransferRoutingDistanceTooLong;
import org.opentripplanner.graph_builder.services.GraphBuilderModule;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.graph.GraphIndex;
import org.opentripplanner.routing.vertextype.TransitStop;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.stream.Collectors;

/**
 * Module used for analyzing the transfers between nearby stops generated by routing via OSM data. It creates
 * annotations both for nearby stops that cannot be routed between and instances where the street routing distance
 * is unusually long compared to the euclidean distance (sorted by the ratio between the two distances). These lists
 * can typically be used to improve the quality of OSM data for transfer purposes. This can take a long time if the
 * transfer distance is long and/or there are many stops to route between.
 */

public class DirectTransferAnalyzer implements GraphBuilderModule {

    private static Logger LOG = LoggerFactory.getLogger(DirectTransferAnalyzer.class);

    final double radiusMeters;

    static final int RADIUS_MULTIPLIER = 5;

    static final int MIN_RATIO_TO_LOG = 2;

    static final int MIN_STREET_DISTANCE_TO_LOG = 100;

    public DirectTransferAnalyzer (double radiusMeters) {
        this.radiusMeters = radiusMeters;
    }

    @Override
    public void buildGraph(Graph graph, HashMap<Class<?>, Object> extra) {
        /* Initialize graph index which is needed by the nearby stop finder. */
        graph.index = new GraphIndex(graph);

        LOG.info("Analyzing transfers (this can be time consuming)...");

        List<TransferInfo> directTransfersTooLong = new ArrayList<>();
        List<TransferInfo> directTransfersNotFound = new ArrayList<>();

        NearbyStopFinder nearbyStopFinderEuclidian = new NearbyStopFinder(graph, radiusMeters, false);
        NearbyStopFinder nearbyStopFinderStreets =
                new NearbyStopFinder(graph, radiusMeters * RADIUS_MULTIPLIER, true);

        for (TransitStop originStop : Iterables.filter(graph.getVertices(), TransitStop.class)) {
            /* Find nearby stops by euclidean distance */
            Map<TransitStop, NearbyStopFinder.StopAtDistance> stopsEuclidean =
                    nearbyStopFinderEuclidian.findNearbyStopsEuclidean(originStop).stream()
                            .collect(Collectors.toMap(t -> t.tstop, t -> t));

            /* Find nearby stops by street distance */
            Map<TransitStop, NearbyStopFinder.StopAtDistance> stopsStreets =
                    nearbyStopFinderStreets.findNearbyStopsViaStreets(originStop).stream()
                            .collect(Collectors.toMap(t -> t.tstop, t -> t));

            /* Get stops found by both street and euclidean search */
            List<TransitStop> stopsConnected =
                    stopsEuclidean.keySet().stream().filter(t -> stopsStreets.keySet().contains(t)
                            && t != originStop)
                            .collect(Collectors.toList());

            /* Get stops found by euclidean search but not street search */
            List<TransitStop> stopsUnconnected =
                    stopsEuclidean.keySet().stream().filter(t -> !stopsStreets.keySet().contains(t)
                            && t != originStop)
                            .collect(Collectors.toList());

            for (TransitStop destStop : stopsConnected) {
                NearbyStopFinder.StopAtDistance euclideanStop = stopsEuclidean.get(destStop);
                NearbyStopFinder.StopAtDistance streetStop = stopsStreets.get(destStop);

                TransferInfo transferInfo = new TransferInfo(
                        originStop,
                        destStop,
                        euclideanStop.dist,
                        streetStop.dist);

                /* Log transfer where the street distance is too long compared to the euclidean distance */
                if (transferInfo.ratio > MIN_RATIO_TO_LOG && transferInfo.streetDistance > MIN_STREET_DISTANCE_TO_LOG) {
                    directTransfersTooLong.add(transferInfo);
                }
            }

            for (TransitStop destStop : stopsUnconnected) {
                NearbyStopFinder.StopAtDistance euclideanStop = stopsEuclidean.get(destStop);

                /* Log transfers that are found by euclidean search but not by street search */
                directTransfersNotFound.add(
                        new TransferInfo(
                                originStop,
                                destStop,
                                euclideanStop.dist,
                                -1));
            }
        }

        /* Sort by street distance to euclidean distance ratio before adding to annotations */
        directTransfersTooLong.sort(Comparator.comparingDouble(TransferInfo::getRatio).reversed());

        for (TransferInfo transferInfo : directTransfersTooLong) {
            graph.addBuilderAnnotation(new TransferRoutingDistanceTooLong(
                    transferInfo.origin,
                    transferInfo.destination,
                    transferInfo.directDistance,
                    transferInfo.streetDistance,
                    transferInfo.ratio
            ));
        }

        /* Sort by origin stop name before adding to annotations */
        directTransfersNotFound.sort(Comparator.comparing(t -> t.origin.getStop().getName()));

        for (TransferInfo transferInfo : directTransfersNotFound) {
            graph.addBuilderAnnotation(new TransferCouldNotBeRouted(
                    transferInfo.origin,
                    transferInfo.destination,
                    transferInfo.directDistance
            ));
        }

        LOG.info("Done analyzing transfers. {} transfers could not be routed and {} transfers had a too long routing" +
                " distance.", directTransfersNotFound.size(), directTransfersTooLong.size());
    }

    @Override
    public void checkInputs() {
        // No inputs
    }

    static class TransferInfo {
        TransitStop origin;
        TransitStop destination;
        double directDistance;
        double streetDistance;
        double ratio;

        public double getRatio() {
            return ratio;
        }

        TransferInfo(
                TransitStop origin,
                TransitStop destination,
                double directDistance,
                double streetDistance) {
            this.origin = origin;
            this.destination = destination;
            this.directDistance = directDistance;
            this.streetDistance = streetDistance;
            this.ratio = directDistance != 0 ? streetDistance / directDistance : 0;
        }
    }
}
