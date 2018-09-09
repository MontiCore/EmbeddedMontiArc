package simulation.environment;

import commons.map.Adjacency;
import commons.map.ControllerContainer;
import commons.map.IAdjacency;
import commons.map.IControllerNode;
import commons.simulation.IPhysicalVehicle;
import commons.simulation.PhysicalObject;
import javafx.geometry.Point3D;
import org.apache.commons.math3.linear.RealVector;
import simulation.environment.geometry.DetailedMapConstructor;
import simulation.environment.geometry.StreetSignPositioner;
import simulation.environment.geometry.osmadapter.GeomStreet;
import simulation.environment.object.TrafficLightSwitcher;
import simulation.environment.osm.IParser;
import simulation.environment.osm.Parser2D;
import simulation.environment.osm.ParserSettings;
import simulation.environment.osm.ZCoordinateGenerator;
import simulation.environment.pedestrians.PedestrianContainer;
import simulation.environment.visualisationadapter.implementation.Node2D;
import simulation.environment.visualisationadapter.interfaces.EnvNode;
import simulation.environment.visualisationadapter.interfaces.EnvStreet;
import simulation.environment.visualisationadapter.interfaces.VisualisationEnvironmentContainer;
import simulation.environment.weather.Weather;
import simulation.environment.weather.WeatherSettings;


import java.util.*;

/**
 * Created by lukas on 02.02.17.
 *
 * The WorldModel to be used
 */
public class WorldModel implements World{
    private static WorldModel ourInstance;

    private static final String defaultMap = "/map_ahornstrasse.osm";

    public static World getInstance() {
        if(ourInstance == null) {
            try {
                ourInstance = new WorldModel(defaultMap);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        return ourInstance;
    }

    public static World init(String map, WeatherSettings weatherSettings) throws Exception {
        ourInstance = new WorldModel(new ParserSettings(map, ParserSettings.ZCoordinates.ALLZERO), weatherSettings);
        return ourInstance;
    }

    public static World init(ParserSettings pSettings, WeatherSettings weatherSettings) throws Exception {
        ourInstance = new WorldModel(pSettings, weatherSettings);
        return ourInstance;
    }

    /**
     * Initialize singleton World instance.
     * 
     * No map parsing done in this method,
     * rather only weather settings are 
     * getting parsed and already prepared
     * visualisation container is set and used.
     * 
     * @param visContainer - already parsed map
     * @param weatherSettings - weather settings
     * @return singleton World instance
     */
    public static World init(VisualisationEnvironmentContainer visContainer, WeatherSettings weatherSettings) {
    	ourInstance = new WorldModel(visContainer, weatherSettings);
    	return ourInstance;
    }
    
    /**
     * Parse only the world map and return
     * its VisualisationEnvironmentContainer.
     * 
     * This method does not initialize the singleton World instance!
     * 
     * @param pSettings - parser settings
     * @return VisualisationEnvironmentContainer object
     * @throws Exception
     */
    public static VisualisationEnvironmentContainer prepareWorldMap(ParserSettings pSettings) throws Exception {
    	return new WorldModel(pSettings).getContainer();
    }
    
    /** 
     * Builds controller-container only. 
     *  
     * Singleton instance is not initialized. 
     *  
     * @param visContainer - pre-built VisualisationEnvironmentContainer 
     * @param wSettings - weather settings 
     * @return ControllerContainer 
     */ 
    public static ControllerContainer prepareControllerContainer(VisualisationEnvironmentContainer visContainer, WeatherSettings wSettings) { 
      return new WorldModel(visContainer, wSettings).getControllerMap(); 
    }

    private VisualisationEnvironmentContainer visualisationContainer;

    private ArrayList<GeomStreet> streets;

    private Weather weather;

    private PedestrianContainer pedContainer;

    private ParserSettings pSettings;

    private ControllerContainer contContainer;

    
    private WorldModel(ParserSettings pSettings, WeatherSettings settings) throws Exception {
        this.pSettings = pSettings;
        parseWorld(pSettings);
        constructGeomStreets();
        positionStreetSigns();
        initWeather(settings);
        constructControllerContainer();
        initPedestrians();
    }

    private WorldModel(String map) throws Exception {
        this.pSettings = new ParserSettings(map, ParserSettings.ZCoordinates.ALLZERO);
        parseWorld(pSettings);
        constructGeomStreets();
        positionStreetSigns();
        initWeather(new WeatherSettings(Weather.SUNSHINE));
        constructControllerContainer();
        initPedestrians();
    }
    
    /**
     * Parse world only, no operations 
     * on weather settings required.
     * 
     * @param pSettings - parser settings
     * @throws Exception
     */
    private WorldModel(ParserSettings pSettings) throws Exception {
    	this.pSettings = pSettings;
        parseWorld(pSettings);
    }

    /**
     * Parse weather settings only.
     * VisualizationContainer is already prepared.
     * 
     * @param visContainer - parsed map
     * @param weatherSettings - weather settings
     */
    private WorldModel(VisualisationEnvironmentContainer visContainer, WeatherSettings weatherSettings) {
		//directly assign visualization container - no map-parsing required
    	this.visualisationContainer = visContainer;
		constructGeomStreets();
        positionStreetSigns();
        initWeather(weatherSettings);
        constructControllerContainer();
        initPedestrians();
	}

	private void initPedestrians() {
        this.pedContainer = new PedestrianContainer(this.streets, 1);
    }

    private void constructControllerContainer() {
        ArrayList<IAdjacency> controllerMap = new ArrayList<>();
        HashMap<Long, Point3D> idToPoint = new HashMap<>();
        for(GeomStreet s : this.streets) {
            List<EnvNode> nodes = ((EnvStreet) (s.getObject())).getNodes();
            for(int i = 0; i < nodes.size() - 1; i++) {
                Node2D n1 = (Node2D) (nodes.get(i));
                Node2D n2 = (Node2D) (nodes.get(i+1));
                controllerMap.add(new Adjacency(n1, n2));

                // If street is not oneWay, then add other direction as well
                if (s.getObject() instanceof EnvStreet) {
                    if (!(((EnvStreet)s.getObject()).isOneWay())) {
                        controllerMap.add(new Adjacency(n2, n1));
                    }
                }

                idToPoint.put(n1.getOsmId(), n1.getPoint());

                if(i == nodes.size() - 2) {
                    idToPoint.put(n2.getOsmId(), n2.getPoint());
                }
            }
        }

        this.contContainer = new ControllerContainer(controllerMap, new DetailedMapConstructor(idToPoint));

    }

    private void positionStreetSigns() {
        StreetSignPositioner.positionStreetSigns(this.streets);
    }

    private void initWeather(WeatherSettings settings) {
       this.weather = new Weather(settings);
    }

    private void parseWorld(ParserSettings pSettings) throws Exception {
        IParser parser = new Parser2D(pSettings);
        parser.parse();
        this.visualisationContainer = parser.getContainer();
    }

    private void constructGeomStreets() {
        this.streets = new ArrayList<>();
        for(EnvStreet street : visualisationContainer.getStreets()) {
            this.streets.add(new GeomStreet(street));
        }
    }

    @Override
    public Number getGround(Number x, Number y, Number z) {
        EnvNode n = new Node2D(x.doubleValue(), y.doubleValue(), z.doubleValue());
        for(GeomStreet minStreet : this.streets) {
            //compute ground on Street
            double streetZ = minStreet.getGround(x.doubleValue(), y.doubleValue(), z.doubleValue());
            //compute node with new z-Coordinate
            Node2D n1 = new Node2D(x.doubleValue(), y.doubleValue(), streetZ);

            //check if node is on street else return z in Environment
            if(minStreet.contains(n1)) {
                return streetZ;
            }
        }
        return ZCoordinateGenerator.getGround(x.doubleValue(),y.doubleValue());
    }

    @Override
    public Number getGroundForNonStreet(Number x, Number y) {
        return ZCoordinateGenerator.getGround(x.doubleValue(), y.doubleValue());
    }

    private GeomStreet getMinimumStreetForNode(EnvNode n) {
        double minDist = Double.MAX_VALUE;
        GeomStreet minStreet = null;

        for(GeomStreet s : this.streets) {
            double tmpDist = s.getDistanceToMiddle(n);
            if(tmpDist < minDist) {
                minStreet = s;
                minDist = tmpDist;
            }

        }
        return minStreet;
    }

    /**
     *
     * @param o
     * @return returns the Street the Vehicle is on
     */
    @Override
    public GeomStreet getStreet(PhysicalObject o){
        EnvNode n = new Node2D(o.getGeometryPos().getEntry(0),o.getGeometryPos().getEntry(1),o.getGeometryPos().getEntry(2));
        GeomStreet street = getMinimumStreetForNode(n);
        return street;
    }

    /**
     *
     * @param n
     * @param numberOfStreets
     * @return returns the n nearest streets for this node
     */
    private List<GeomStreet> getMinimumStreetsForNode(EnvNode n, int numberOfStreets) {
        TreeMap<Double, GeomStreet> streetDistances = new TreeMap<>();

        ArrayList<GeomStreet> result = new ArrayList<>();

        for(GeomStreet s : this.streets) {
            streetDistances.put(s.getDistanceToMiddle(n), s);
        }

        Iterator<Double> iter = streetDistances.keySet().iterator();
        int i = 0;
        while(iter.hasNext() && i < numberOfStreets) {
            result.add(streetDistances.get(iter.next()));
        }


        return result;
    }

    @Override
    public Number getDistanceToMiddleOfStreet(PhysicalObject o) {
        EnvNode n = new Node2D(o.getGeometryPos().getEntry(0),o.getGeometryPos().getEntry(1),o.getGeometryPos().getEntry(2));
        GeomStreet minStreet = getMinimumStreetForNode(n);
        return minStreet.getDistanceToMiddle(o);
    }

    @Override
    public Number getDistanceToLeftStreetBorder(IPhysicalVehicle v) {
        EnvNode n = new Node2D(v.getGeometryPos().getEntry(0),v.getGeometryPos().getEntry(1),v.getGeometryPos().getEntry(2));
        GeomStreet minStreet = getMinimumStreetForNode(n);

        return minStreet.getDistanceToLeft(v);
    }

    @Override
    public Number getDistanceFrontLeftWheelToLeftStreetBorder(IPhysicalVehicle v) {
        RealVector pos = v.getFrontLeftWheelGeometryPosition();

        EnvNode n = new Node2D(pos.getEntry(0),pos.getEntry(1),pos.getEntry(2));
        GeomStreet minStreet = getMinimumStreetForNode(n);

        return minStreet.getDistanceToLeft(v);
    }

    @Override
    public Number getDistanceBackLeftWheelToLeftStreetBorder(IPhysicalVehicle v) {
        RealVector pos = v.getBackLeftWheelGeometryPosition();

        EnvNode n = new Node2D(pos.getEntry(0),pos.getEntry(1),pos.getEntry(2));
        GeomStreet minStreet = getMinimumStreetForNode(n);

        return minStreet.getDistanceToLeft(v);
    }

    @Override
    public Number getDistanceLeftFrontToStreetBorder(IPhysicalVehicle v){
        RealVector pos = v.getBackLeftWheelGeometryPosition();

        EnvNode n = new Node2D(pos.getEntry(0),pos.getEntry(1),pos.getEntry(2));
        GeomStreet minStreet = getMinimumStreetForNode(n);

        return minStreet.getDistancetoFrontLeft(v);
    }

    @Override
    public Number getDistanceRightFrontToStreetBorder(IPhysicalVehicle v){
        RealVector pos = v.getBackLeftWheelGeometryPosition();

        EnvNode n = new Node2D(pos.getEntry(0),pos.getEntry(1),pos.getEntry(2));
        GeomStreet minStreet = getMinimumStreetForNode(n);

        return minStreet.getDistancetoFrontRight(v);
    }

    @Override
    public Number getDistanceToRightStreetBorder(IPhysicalVehicle v) {
        EnvNode n = new Node2D(v.getGeometryPos().getEntry(0),v.getGeometryPos().getEntry(1),v.getGeometryPos().getEntry(2));
        GeomStreet minStreet = getMinimumStreetForNode(n);

        return minStreet.getDistanceToRight(v);
    }

    @Override
    public Number getDistanceFrontRightWheelToRightStreetBorder(IPhysicalVehicle v) {
        RealVector pos = v.getFrontRightWheelGeometryPosition();

        EnvNode n = new Node2D(pos.getEntry(0),pos.getEntry(1),pos.getEntry(2));
        GeomStreet minStreet = getMinimumStreetForNode(n);

        return minStreet.getDistanceToLeft(v);
    }

    @Override
    public Number getDistanceBackRightWheelToRightStreetBorder(IPhysicalVehicle v) {
        RealVector pos = v.getBackRightWheelGeometryPosition();

        EnvNode n = new Node2D(pos.getEntry(0),pos.getEntry(1),pos.getEntry(2));
        GeomStreet minStreet = getMinimumStreetForNode(n);

        return minStreet.getDistanceToLeft(v);
    }

    @Override
    public VisualisationEnvironmentContainer getContainer() throws Exception {
        return this.visualisationContainer;
    }

    @Override
    public boolean isItRaining() {
        return this.weather.isRain();
    }

    @Override
    public double getWeather() {
        return this.weather.getWeather();
    }

    @Override
    public ControllerContainer getControllerMap() {
        return this.contContainer;
    }

    @Override
    public PedestrianContainer getPedestrianContainer() {
        return this.pedContainer;
    }

    @Override
    public Point3D spawnOnStreet(Number x, Number y, Number z, boolean rightLane) {
        EnvNode n = new Node2D(x.doubleValue(), y.doubleValue(), z.doubleValue());
        GeomStreet minStreet = getMinimumStreetForNode(n);
        return minStreet.spawnCar(rightLane, n.getPoint());
    }

    @Override
    public Point3D spawnNotOnStreet(Number x, Number y, Number z) {
        EnvNode n = new Node2D(x.doubleValue(), y.doubleValue(), z.doubleValue());
        GeomStreet minStreet = getMinimumStreetForNode(n);
        double newZ = minStreet.getGround(x.doubleValue(), y.doubleValue(), z.doubleValue());
        if(!minStreet.contains(new Node2D(x.doubleValue(), y.doubleValue(), newZ))) {
            newZ = ZCoordinateGenerator.getGround(x.doubleValue(), y.doubleValue());
            return new Point3D(x.doubleValue(), y.doubleValue(), newZ);
        }

        return null;
    }

    @Override
    public List<Long> getChangedTrafficSignals() {
        List<Long> result = new ArrayList<>();
        for(TrafficLightSwitcher t : TrafficLightSwitcher.getSwitcher()) {
            result.addAll(t.getChangedState());
        }
        return result;
    }

    @Override
    public IControllerNode getRandomNode() {
        if(visualisationContainer == null) {
            return null;
        } else {
            Random r = new Random();
            ArrayList<EnvStreet> streets = new ArrayList<>(visualisationContainer.getStreets());
            int street = r.nextInt(streets.size());
            return streets.get(street).getNodes().get(r.nextInt(streets.get(street).getNodes().size()));
        }
    }


}
