package org.opentripplanner.ext.fares.impl;

import static java.util.Objects.isNull;
import static java.util.Objects.nonNull;

import com.google.common.collect.Multimap;
import java.io.Serializable;
import java.util.Collection;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.opentripplanner.model.FareLegRule;
import org.opentripplanner.model.FareProduct;
import org.opentripplanner.model.plan.Itinerary;
import org.opentripplanner.model.plan.ScheduledTransitLeg;
import org.opentripplanner.transit.model.framework.FeedScopedId;
import org.opentripplanner.transit.model.site.StopLocation;

public final class GtfsFaresV2Service implements Serializable {

  private final List<FareLegRule> legRules;
  private final Multimap<FeedScopedId, String> stopAreas;
  private final Set<String> networksWithRules;
  private final Set<String> fromAreasWithRules;
  private final Set<String> toAreasWithRules;

  public GtfsFaresV2Service(List<FareLegRule> legRules, Multimap<FeedScopedId, String> stopAreas) {
    this.legRules = legRules;
    this.networksWithRules = findNetworksWithRules(legRules);
    this.fromAreasWithRules = findAreasWithRules(legRules, FareLegRule::fromAreaId);
    this.toAreasWithRules = findAreasWithRules(legRules, FareLegRule::toAreadId);
    this.stopAreas = stopAreas;
  }

  public ProductResult getProducts(Itinerary itinerary) {
    var coveringItinerary = itinerary
      .getTransitLegs()
      .stream()
      .flatMap(this::getLegProducts)
      .filter(p -> p.coversItinerary(itinerary))
      .distinct()
      .toList();

    return new ProductResult(coveringItinerary);
  }

  private static Set<String> findAreasWithRules(
    List<FareLegRule> legRules,
    Function<FareLegRule, String> getArea
  ) {
    return legRules.stream().map(getArea).filter(Objects::nonNull).collect(Collectors.toSet());
  }

  private static Set<String> findNetworksWithRules(Collection<FareLegRule> legRules) {
    return legRules
      .stream()
      .map(FareLegRule::networkId)
      .filter(Objects::nonNull)
      .collect(Collectors.toSet());
  }

  private Stream<FareProduct> getLegProducts(ScheduledTransitLeg leg) {
    return legRules
      .stream()
      // make sure that you only get rules for the correct feed
      .filter(legRule -> leg.getAgency().getId().getFeedId().equals(legRule.feedId()))
      .filter(rule -> filterByNetworkId(leg, rule))
      // apply only those fare leg rules which have the correct area ids
      // if area id is null, the rule applies to all legs UNLESS there is another rule that
      // covers this area
      .filter(rule -> filterByArea(leg.getFrom().stop, rule.fromAreaId(), fromAreasWithRules))
      .filter(rule -> filterByArea(leg.getTo().stop, rule.toAreadId(), toAreasWithRules))
      .map(FareLegRule::fareProduct);
  }

  private boolean filterByArea(StopLocation stop, String areaId, Set<String> areasWithRules) {
    var stopAreas = this.stopAreas.get(stop.getId());
    return (
      (isNull(areaId) && stopAreas.stream().noneMatch(areasWithRules::contains)) ||
      (nonNull(areaId) && stopAreas.contains(areaId))
    );
  }

  /**
   * Get the fare products that match the network_id. If the network id of the product is null it
   * depends on the presence/absence of other rules with that network id.
   */
  private boolean filterByNetworkId(ScheduledTransitLeg leg, FareLegRule rule) {
    return (
      (isNull(rule.networkId()) && !networksWithRules.contains(leg.getRoute().getNetworkId())) ||
      Objects.equals(rule.networkId(), leg.getRoute().getNetworkId())
    );
  }
}

record ProductResult(List<FareProduct> productsCoveringItinerary) {}
