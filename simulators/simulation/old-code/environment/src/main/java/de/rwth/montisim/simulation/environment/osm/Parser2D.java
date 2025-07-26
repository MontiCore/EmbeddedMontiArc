/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.osm;

import de.topobyte.osm4j.core.access.OsmInputException;
import de.topobyte.osm4j.core.access.OsmReader;
import de.topobyte.osm4j.core.dataset.InMemoryMapDataSet;
import de.topobyte.osm4j.core.dataset.MapDataSetLoader;
import de.topobyte.osm4j.core.model.iface.OsmNode;
import de.topobyte.osm4j.core.model.iface.OsmTag;
import de.topobyte.osm4j.core.model.iface.OsmWay;
import de.topobyte.osm4j.core.model.util.OsmModelUtil;
import de.topobyte.osm4j.core.resolve.EntityNotFoundException;
import de.topobyte.osm4j.xml.dynsax.OsmXmlReader;
import org.apache.commons.lang3.StringUtils;
import de.rwth.montisim.commons.utils.Vec3;
import org.w3c.dom.Document;
import de.rwth.montisim.simulation.environment.object.ChargingStation;
import de.rwth.montisim.simulation.environment.osm.Dto.OSMNode;
import de.rwth.montisim.simulation.environment.visualisationadapter.implementation.*;
import de.rwth.montisim.simulation.environment.visualisationadapter.interfaces.*;

import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.InputStream;
import java.util.*;

/**
 * Created by lukas on 08.01.17.
 * Parses OSM-Data using osm4j
 * Update by Florisa on 31.01.19.
 * Update by Danilo on 07.02.19
 */
public class Parser2D implements IParser {
    private InMemoryMapDataSet dataSet;

    private IntersectionFinder finder;

    private IntersectionMapper mapper;

    private HashSet<EnvStreet> streets;
    private HashSet<Building> buildings;
    private HashSet<Waterway> waterway;
    private HashSet<ChargingStation> chargingStations;

    private String filePath;
    private String mapName;

    private InputStream in;

    EnvironmentContainerConverter converter;
    private EnvironmentContainer2D container;

    private ParserSettings.ZCoordinates z;

    private EnvironmentContainer2D containerM;

    private double minLong = Double.MAX_VALUE;
    private double minLat = Double.MAX_VALUE;

    public Parser2D(ParserSettings pSettings) {
        this.in = pSettings.in;
        this.z = pSettings.z;
        this.mapName = pSettings.mapName;
        init();
    }

    private void init() {
        this.streets = new HashSet<>();
        this.buildings = new HashSet<>();
        this.waterway = new HashSet<>();
        this.chargingStations = new HashSet<>();
    }

    /**
     * parses the data, finds intersections, maps the intersection to the streets and converts the container into kilometric units
     * @throws Exception
     */
    public void parse() throws Exception {
        if (this.in == null) {
            if (!StringUtils.isBlank(this.filePath)) {
                this.in = new FileInputStream(filePath);
            } else {
                throw new IllegalArgumentException("No Input");
            }
        }
        /*
        //Automatic call of OSM-Data. Currently not used cause of manually changes in Map-Data.
        OSMConnector connector = new OSMConnector();
        //Call Area you want to load. Further instructions in class doc.

        //TODO: For now only RWTH-AAchen coordinates. Can be extended with all possible coordinates.
        Document mapData = connector.getXML(6.05892134,50.77828146,10);

        //Create temp file for loaded data.
        DOMSource source = new DOMSource(mapData);
        FileWriter writer = new FileWriter(new File("/tmp/output.xml"));
        StreamResult result = new StreamResult(writer);

        TransformerFactory transformerFactory = TransformerFactory.newInstance();
        Transformer transformer = transformerFactory.newTransformer();
        transformer.transform(source, result);

        this.in = new FileInputStream("/tmp/output.xml");
        */

        executeParsing();

    }

    public void parseCertainArea(double lon, double lat, double vicinityRange) throws Exception {
        //Automatic call of OSM-Data. Currently not used cause of manually changes in Map-Data.
        OSMConnector connector = new OSMConnector();
        //Call Area you want to load. Further instructions in class doc.

        //TODO: For now only RWTH-AAchen coordinates. Can be extended with all possible coordinates.
        Document mapData = connector.getXML(lon, lat, vicinityRange);
        //Create temp file for loaded data.
        DOMSource source = new DOMSource(mapData);
        FileWriter writer = new FileWriter(new File("/tmp/output.xml"));
        StreamResult result = new StreamResult(writer);

        TransformerFactory transformerFactory = TransformerFactory.newInstance();
        Transformer transformer = transformerFactory.newTransformer();
        transformer.transform(source, result);

        this.in = new FileInputStream("/tmp/output.xml");

        executeParsing();
    }

    private void executeParsing() throws Exception {
        try {
            OsmReader reader = new OsmXmlReader(in, false);
            this.dataSet = MapDataSetLoader.read(reader, true, true, true);

            // Manually filter dataSet, can not really use OsmTagFilter for that since it is too inflexible
            // Compute minLong and minLat on all unfiltered OsmWays with key highway
            // first to ensure that the same coordinates are generated by conversion from longitude / latitude to metric units
            for (OsmWay way : this.dataSet.getWays().valueCollection()) {

                Map<String, String> tags = OsmModelUtil.getTagsAsMap(way);
                String highway = tags.get("highway");
                String building = tags.get("building");
                String waterway = tags.get("waterway");
                String surface = tags.get("surface");

                if (highway == null && building == null && waterway == null) {
                    continue;
                }

                for (int i = 0; i < way.getNumberOfNodes(); i++) {
                    OsmNode node = this.dataSet.getNode(way.getNodeId(i));

                    double nLat = node.getLatitude();
                    double nLong = node.getLongitude();

                    if (nLat <= minLat) {
                        minLat = nLat;
                    }

                    if (nLong <= minLong) {
                        minLong = nLong;
                    }
                }
            }

            // Only OsmWays are used after this step, so its fine to leave Nodes / Relations unfiltered
            LinkedList<Long> removeListOsmWayMapIDs = new LinkedList<>();

            for (Long osmWayMapID : this.dataSet.getWays().keys()) {
                OsmWay way = this.dataSet.getWay(osmWayMapID);

                for (int i = 0; i < way.getNumberOfTags(); ++i) {
                    OsmTag tag = way.getTag(i);
                    String key = tag.getKey();
                    String val = tag.getValue();

                    // Filter key:highway with values in: footway, bridleway, steps, path, cycleway, pedestrian, track, elevator
                    if (key.equals("highway") && (val.equals("footway") || val.equals("bridleway") || val.equals("steps") || val.equals("path") || val.equals("cycleway") || val.equals("pedestrian") || val.equals("track") || val.equals("elevator"))) {
                        removeListOsmWayMapIDs.add(osmWayMapID);
                        break;
                    }
                }
            }

            // Remove marked IDs from dataset
            for (Long removeOsmWayMapID : removeListOsmWayMapIDs) {
                this.dataSet.getWays().remove(removeOsmWayMapID);
            }

        } catch (OsmInputException e) {
            e.printStackTrace();
        }

        findIntersections();
        mapIntersections();
        try {
            parseObjects();
        } catch (EntityNotFoundException e) {
            e.printStackTrace();
        }

        parseChargingStations();
        buildContainer();
        addSomeRandomTrees();
        generateZCoordinates();

        // Convert values to meter
        if (minLong != Double.MAX_VALUE && minLat != Double.MAX_VALUE) {
            converter = new EnvironmentContainerConverter(this.container, minLong, minLat);
        } else {
            converter = new EnvironmentContainerConverter(this.container);
        }
        this.containerM = converter.getContainer();

        // Set relevant data for height map
        ZCoordinateGenerator.setLongLatToMetersConverter(converter.getApproximateConverter());
        this.containerM.setHeightMap(ZCoordinateGenerator.getHeightMap());
        this.containerM.setHeightMapDelta(ZCoordinateGenerator.getHeightMapDeltaX(), ZCoordinateGenerator.getHeightMapDeltaY());
        this.containerM.setHeightMapMinMax(ZCoordinateGenerator.getHeightMapMinPoint(), ZCoordinateGenerator.getHeightMapMaxPoint());

        addStreetSigns();
    }

    private void parseChargingStations() {
        // Each charging station in OSM are represented by a node with tag amenity=charging_station
        // We iterate over all nodes in the map to find charging stations
        for (OsmNode node : dataSet.getNodes().valueCollection()) {
            Map<String, String> tags = OsmModelUtil.getTagsAsMap(node);
            String amenity = tags.get("amenity");

            if (amenity != null && amenity.equals("charging_station")) {
                String capacity = tags.get("capacity");
                if (capacity == null) {
                    capacity = "1";
                }
                String name = tags.get("name");
                if (name == null) {
                    name = "chargingStation";
                }

                this.chargingStations.add(new ChargingStation(
                        node.getId(),
                        new Node2D(node.getLongitude(), node.getLatitude(), 0, node.getId()),
                        Integer.parseInt(capacity),
                        name)
                );
            }
        }
    }

    private void addStreetSigns() {
        // Disable street signs for now, they are not supported by computer vision / controller yet
        // StreetSignGenerator.generateStreetSigns(containerM);
    }

    private void generateZCoordinates() {
        ZCoordinateGenerator.generateZCoordinates(container, this.z);
        container.setHeightMap(ZCoordinateGenerator.getHeightMap());
    }

    private void addSomeRandomTrees() {
        //TODO: implement here sampling of trees from some distribution e.g. Gaussian
        ArrayList<EnvNode> trees = new ArrayList<>();
        trees.add(new Node2D(10, 20, 30));
        trees.add(new Node2D(40, 50, 60));
        this.container.setTrees(trees);
    }

    private void buildContainer() {

        this.container = new simulation.environment.visualisationadapter.implementation.EnvironmentContainer2D(
                new Bounds2D(this.dataSet.getBounds().getLeft(), this.dataSet.getBounds().getRight(), this.dataSet.getBounds().getBottom(), this.dataSet.getBounds().getTop(), 0, 0),
                this.streets, this.buildings, this.waterway, this.chargingStations);

    }

    private void findIntersections() {
        finder = IntersectionFinder.getInstance();
        finder.findIntersections(dataSet);
    }

    private void mapIntersections() {
        this.mapper = new IntersectionMapper(dataSet, finder.getIntersections());
    }

    /**
     * uses only streets out of the osm-data. more was not possible due to the precision of the kilometric conversion
     * @throws EntityNotFoundException
     */
    private void parseObjects() throws EntityNotFoundException {
        for (OsmWay way : dataSet.getWays().valueCollection()) {
            Map<String, String> tags = OsmModelUtil.getTagsAsMap(way);
            String highway = tags.get("highway");
            String building = tags.get("building");
            String waterway = tags.get("waterway");
            String surface = tags.get("surface");


            //  System.out.println("PARSE OBJECTS FUNCTION  "+highway);
            //  System.out.println(waterway);

            if (highway != null) {
                // Check if way is marked as oneWayRoad
                boolean isOneWay = false;
                String oneWayRoad = tags.get("oneway");
                if (oneWayRoad != null && oneWayRoad.equals("yes")) {
                    isOneWay = true;
                }

                constructStreet(way, isOneWay, highway, surface);
            }
            if (building != null) {
                constructBuilding(way, building);
            }

            if (waterway != null) {
                constructWaterway(way, waterway);
            }
        }
    }

    private void constructWaterway(OsmWay way, String waterway) throws EntityNotFoundException {


        List<EnvNode> nodes = new ArrayList<>();
        for (int i = 0; i < way.getNumberOfNodes(); i++) {

            OsmNode node = dataSet.getNode(way.getNodeId(i));

            nodes.add(new Node2D(node.getLongitude(), node.getLatitude(), 0, way.getNodeId(i)));
        }
        this.waterway.add(new Waterway2D(nodes, way.getId()));
    }


    private void constructBuilding(OsmWay way, String building) throws EntityNotFoundException {

        List<EnvNode> nodes = new ArrayList<>();
        for (int i = 0; i < way.getNumberOfNodes(); i++) {
            OsmNode node = dataSet.getNode(way.getNodeId(i));
            nodes.add(new Node2D(node.getLongitude(), node.getLatitude(), 0, way.getNodeId(i)));
        }

        this.buildings.add(new BuildingImpl(nodes, way.getId()));


    }

    /**
     * construct a Street2D from a given OsmWay object
     * @param way
     * @param isOneWay
     * @param highway
     * @throws EntityNotFoundException
     */
    private void constructStreet(OsmWay way, boolean isOneWay, String highway, String surface) throws EntityNotFoundException {
        List<EnvNode> nodes = new ArrayList<>();

        for (int i = 0; i < way.getNumberOfNodes(); i++) {
            OsmNode node = dataSet.getNode(way.getNodeId(i));
            nodes.add(new Node2D(node.getLongitude(), node.getLatitude(), 0, way.getNodeId(i)));
        }

        this.streets.add(new Street2D(nodes, 50.d, mapper.getIntersectionsForWay(way), way.getId(), isOneWay, parseStreetType(highway), parseStreetPavement(surface)));
    }

    private void constructStreet(OsmWay way, boolean isOneWay, String highway) throws EntityNotFoundException {
        List<EnvNode> nodes = new ArrayList<>();

        for (int i = 0; i < way.getNumberOfNodes(); i++) {
            OsmNode node = dataSet.getNode(way.getNodeId(i));
            nodes.add(new Node2D(node.getLongitude(), node.getLatitude(), 0, way.getNodeId(i)));
        }

        this.streets.add(new Street2D(nodes, 50.d, mapper.getIntersectionsForWay(way), way.getId(), isOneWay, parseStreetType(highway)));
    }

    @Override
    public Collection<EnvStreet> getStreets() {
        return this.streets;
    }

    @Override
    public Collection<Building> getBuildings() {
        return this.buildings;
    }

    @Override
    public Collection<Waterway> getWaterways() {
        return this.waterway;
    }

    public Collection<ChargingStation> getChargingStations() {
        return this.chargingStations;
    }

    public InMemoryMapDataSet getDataSet() {
        return this.dataSet;
    }


    public EnvironmentContainerConverter getConverter() {
        return converter;
    }

    public VisualisationEnvironmentContainer getContainer() {
        return this.containerM;
    }

    /**
     * converts String streettype in enum
     * @param s as streettype
     */
    public EnvStreet.StreetTypes parseStreetType(String s) {
        if (s.equals("motorway")) {
            return EnvStreet.StreetTypes.MOTORWAY;
        } else if (s.equals("unclassified") || s.equals("tertiary") || s.equals("trunk_link")) {
            return EnvStreet.StreetTypes.A_ROAD;
        } else if (s.equals("residential") || s.equals("service")) {
            return EnvStreet.StreetTypes.STREET;
        } else if (s.equals("living_street")) {
            return EnvStreet.StreetTypes.LIVING_STREET;
        } else {
            return EnvStreet.StreetTypes.A_ROAD;    //Default
        }
    }


    /**
     * converts String streetpavement in enum
     * @param s as streetpavement
     */
    public EnvStreet.StreetPavements parseStreetPavement(String s) {
        if (s != null) {
            if (s.equals("paved")) {
                return EnvStreet.StreetPavements.PAVED;
            } else if (s.equals("unpaved")) {
                return EnvStreet.StreetPavements.UNPAVED;
            } else if (s.equals("asphalt") || s.equals("concrete")) {
                return EnvStreet.StreetPavements.QUALITY;
            } else if (s.equals("cobblestone") || s.equals("sett") || s.equals("paving_stone")) {
                return EnvStreet.StreetPavements.STONE;
            } else if (s.equals("compacted") || s.equals("gravel") || s.equals("dirt")) {
                return EnvStreet.StreetPavements.DIRT;
            } else {
                return EnvStreet.StreetPavements.GRASS;    //als default gut wählbar?
            }
        } else {
            return EnvStreet.StreetPavements.PAVED;
        }
    }

    public String getMapName() {
        return this.mapName;
    }
}