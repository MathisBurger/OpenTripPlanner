package org.opentripplanner.transit.raptor.speed_test.options;

import static org.opentripplanner.standalone.config.routerequest.RouteRequestConfig.mapRouteRequest;

import com.fasterxml.jackson.databind.JsonNode;
import java.io.File;
import java.net.URI;
import java.time.LocalDate;
import java.time.ZoneId;
import org.opentripplanner.routing.api.request.RouteRequest;
import org.opentripplanner.standalone.config.framework.file.ConfigFileLoader;
import org.opentripplanner.standalone.config.framework.json.NodeAdapter;
import org.opentripplanner.standalone.config.routerconfig.TransitRoutingConfig;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class SpeedTestConfig {

  private static final Logger LOG = LoggerFactory.getLogger(SpeedTestConfig.class);
  public static final String FILE_NAME = "speed-test-config.json";

  private final NodeAdapter adapter;

  /**
   * The test date is the date used for all test cases. The default value is today.
   */
  public final LocalDate testDate;

  /** The speed test run all its test on an existing pre-build graph. */
  public final URI graph;

  public final String feedId;

  public final TransitRoutingConfig transitRoutingParams;
  public final RouteRequest request;

  public SpeedTestConfig(JsonNode node) {
    this(new NodeAdapter(node, FILE_NAME));
  }

  public SpeedTestConfig(NodeAdapter adapter) {
    this.adapter = adapter;
    testDate = adapter.of("testDate").asDateOrRelativePeriod("PT0D", ZoneId.of("UTC"));
    graph = adapter.of("graph").asUri(null);
    feedId = adapter.of("feedId").asString();
    transitRoutingParams = new TransitRoutingConfig("tuningParameters", adapter);
    request = mapRouteRequest(adapter.of("routingDefaults").asObject());
    adapter.logAllUnusedParameters(LOG::warn);
  }

  public static SpeedTestConfig config(File dir) {
    var json = ConfigFileLoader.of().withConfigDir(dir).loadFromFile(FILE_NAME);
    SpeedTestConfig config = new SpeedTestConfig(json);
    LOG.info("SpeedTest config loaded: {}", config);
    return config;
  }

  @Override
  public String toString() {
    return adapter.toPrettyString();
  }
}
