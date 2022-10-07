/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.environment.osmmap;

import de.rwth.montisim.commons.utils.XmlTraverser;
import de.rwth.montisim.commons.utils.BuildObject;
import de.rwth.montisim.commons.utils.Coordinates;
import de.rwth.montisim.commons.utils.StringRef;

import java.io.File;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Vector;
import java.util.logging.Logger;


public class OsmMap implements BuildObject {
    public static final String CONTEXT_KEY = "osm_map";

    public class Tagged {
        // TODO only allocate tags if used
        public HashMap<Integer, Integer> tags = new HashMap<>();

        public String getTag(String tagKey) {
            Integer i = stringId.get(tagKey);
            if (i == null) return null;
            Integer v = tags.get(i);
            if (v == null) return null;
            return getString(v);
        }

        public String getTag(StringId str) {
            Integer v = tags.get(str.id);
            if (v == null) return null;
            return getString(v);
        }

        public int getTagId(String tagKey) {
            Integer i = stringId.get(tagKey);
            if (i == null) return -1;
            Integer v = tags.get(i);
            if (v == null) return -1;
            return v;
        }

        public int getTagId(StringId str) {
            Integer v = tags.get(str.id);
            if (v == null) return -1;
            return v;
        }
    }

    public class Way extends Tagged {
        public final long osmID;
        // Local ID is position in Node table. (Might be null)
        public Vector<Integer> nodesLocalID = new Vector<>();

        public Way(long osmID) {
            this.osmID = osmID;
        }
    }

    public class Node extends Tagged {
        public final long osmID;
        public final Coordinates coords;

        Node(Coordinates coords, long osmID) {
            this.coords = coords;
            this.osmID = osmID;
        }
    }

    /*
        String table system:
        All strings are stored inside the stringTable,
        all are referenced by ids.
    */
    Vector<String> stringTable = new Vector<>();
    HashMap<String, Integer> stringId = new HashMap<>();

    int getStringId(String str) {
        Integer i = stringId.get(str);
        if (i != null) return i;
        int id = stringTable.size();
        stringTable.add(str);
        stringId.put(str, id);
        return id;
    }

    String getString(int strId) {
        return stringTable.elementAt(strId);
    }

    public class StringId {
        public final String str;
        public final int id;

        public StringId(String str) {
            this.str = str;
            this.id = getStringId(str);
        }
    }

    /*
        The different tags and values that are relevant for creating the Map.
    */

    public class Tag {
        public final StringId NAME = new StringId("name");
        public final StringId ONEWAY = new StringId("oneway");
        public final StringId LANES = new StringId("lanes");
        public final StringId AREA = new StringId("area");
        public final StringId HIGHWAY = new StringId("highway");
        public final StringId BUILDING = new StringId("building");
        public final StringId HEIGHT = new StringId("height");
        public final StringId BUILDING_LEVELS = new StringId("building:levels");
        public final StringId MAX_SPEED = new StringId("maxspeed");

        private Tag() {
        }
    }

    public class Highway {
        // https://wiki.openstreetmap.org/wiki/Key:highway
        // Principal roads
        public final StringId MOTORWAY = new StringId("motorway");
        public final StringId TRUNK = new StringId("trunk");
        public final StringId PRIMARY = new StringId("primary");
        public final StringId SECONDARY = new StringId("secondary");
        public final StringId TERTIARY = new StringId("tertiary");
        public final StringId UNCLASSIFIED = new StringId("unclassified");
        public final StringId RESIDENTIAL = new StringId("residential");
        // Link roads
        public final StringId MOTORWAY_LINK = new StringId("motorway_link");
        public final StringId TRUNK_LINK = new StringId("trunk_link");
        public final StringId PRIMARY_LINK = new StringId("primary_link");
        public final StringId SECONDARY_LINK = new StringId("secondary_link");
        public final StringId TERTIARY_LINK = new StringId("tertiary_link");
        // Special road types
        public final StringId LIVING_STREET = new StringId("living_street");
        public final StringId SERVICE = new StringId("service");
        public final StringId PEDESTRIAN = new StringId("pedestrian");
        public final StringId TRACK = new StringId("track");
        public final StringId RACEWAY = new StringId("raceway");
        public final StringId ROAD = new StringId("road"); // For roads of unknown type

        private Highway() {
        }
    }

    public final StringId VALUE_YES = new StringId("yes");
    public final StringId VALUE_NO = new StringId("no");

    public final Tag TAG = new Tag();
    public final Highway HIGHWAY = new Highway();
    HashSet<Integer> roadTags = new HashSet<>();

    /*
        OsmMap data.
    */
    public final String name;

    // Can contain null entries
    public Vector<Node> nodeTable = new Vector<>();
    public Vector<Way> wayTable = new Vector<>();

    public HashMap<Long, Integer> nodeLocalIDByOsmID = new HashMap<>();
    //public HashMap<Long, Integer> wayLocalIDByOsmID = new HashMap<>();

    private int addNode(Node node) {
        Integer i = nodeLocalIDByOsmID.get(node.osmID);
        if (i != null) {
            if (nodeTable.elementAt(i) != null) {
                Logger.getGlobal().warning("OSM Node " + node.osmID + " already added.");
            }
            nodeTable.set(i, node);
            return i;
        } else {
            int id = nodeTable.size();
            nodeLocalIDByOsmID.put(node.osmID, id);
            nodeTable.add(node);
            return id;
        }
    }

    public Node getNode(long osmId) {
        Integer i = nodeLocalIDByOsmID.get(osmId);
        if (i == null) return null;
        return nodeTable.elementAt(i);
    }

    // Returns a local id for the node even if the node itself was not encountered yet and added.
    private int getNodeID(long osmID) {
        Integer i = nodeLocalIDByOsmID.get(osmID);
        if (i != null) return i;
        int id = nodeTable.size();
        nodeLocalIDByOsmID.put(osmID, id);
        nodeTable.add(null);
        return id;
    }

    private int addWay(Way way) {
        int id = wayTable.size();
        wayTable.add(way);
        return id;
    }

    public Coordinates min_corner = new Coordinates(0, 0);
    public Coordinates mid_point = new Coordinates(0, 0);
    public Coordinates max_corner = new Coordinates(0, 0);
    public Coordinates size = new Coordinates(0, 0);

    public OsmMap(String name, File file) throws Exception {
        this.name = name;
        getRoadTagsById();
        parseMap(new XmlTraverser().fromFile(file));
    }

    private void parseMap(XmlTraverser p) throws Exception {
        long start = System.nanoTime();

        //Find osm tag
        boolean foundOsm = false;
        for (StringRef tag : p.tags()) {
            if (tag.equalsIgnoreCase("osm")) {
                foundOsm = true;
                break;
            }
        }
        if (!foundOsm) throw new Exception("Could not find <osm> tag");

        //Ex: <osm version="0.6" generator="CGImap 0.7.5 (14605 thorn-01.openstreetmap.org)" copyright="OpenStreetMap and contributors" attribution="http://www.openstreetmap.org/copyright" license="http://opendatacommons.org/licenses/odbl/1-0/">

        //Search the "version" attribute from the <osm> tag.
        boolean foundVersion = false;
        for (StringRef attrib : p.attribs()) {
            if (attrib.equalsIgnoreCase("version")) {
                foundVersion = true;
                StringRef version = p.getAttributeValue();
                if (!version.equalsIgnoreCase("0.6")) throw new Exception("Unsupported osm version: " + version);
                break;
            }
        }
        //Validate version
        if (!foundVersion) throw new Exception("Could not find osm version attribute");

        //Enter <osm> tag
        p.enterTag();
        //Check all tags from the <osm> tag.
        for (StringRef tag : p.tags()) {
            if (tag.equalsIgnoreCase("node")) {
                //Ex: <node id="6152869602" visible="true" version="1" changeset="65685715" timestamp="2018-12-22T05:39:15Z" user="adjuva" uid="92274" lat="50.7802316" lon="6.0716610"/>
                long id = 0;
                boolean got_id = false;
                double lat = 0;
                double lon = 0;
                for (StringRef attrib : p.attribs()) {
                    if (attrib.equalsIgnoreCase("id")) {
                        got_id = true;
                        id = p.getAttributeValue().asLong();
                    } else if (attrib.equalsIgnoreCase("lat")) {
                        lat = p.getAttributeValue().asDouble();
                    } else if (attrib.equalsIgnoreCase("lon")) {
                        lon = p.getAttributeValue().asDouble();
                    }
                }
                if (!got_id) continue; // Simply ignore if no osmID
                Node node = new Node(new Coordinates(lon, lat), id);

                //Check node tags
                if (p.enterTag()) {
                    for (StringRef tag2 : p.tags()) {
                        if (tag2.equalsIgnoreCase("tag")) {
                            //Ex: <tag k="railway" v="signal"/>
                            String key = null;
                            String value = null;
                            for (StringRef att : p.attribs()) {
                                if (att.equalsIgnoreCase("k")) {
                                    key = p.getAttributeValue().getRawString();
                                } else if (att.equalsIgnoreCase("v")) {
                                    value = p.getAttributeValue().getRawString();
                                }
                            }
                            if (key != null && value != null) {
                                //node.tags.put(get_tag_id(key), value);
                                node.tags.put(getStringId(key), getStringId(value));
                            }
                        }
                    }
                }
                p.exitTag();

                addNode(node);
            } else if (tag.equalsIgnoreCase("way")) {
                //Ex: <way id="5168924" visible="true" version="23" changeset="48959749" timestamp="2017-05-24T23:00:23Z" user="Steffen van Bergerem" uid="61868">
                long id = 0;
                boolean got_id = false;
                for (StringRef attrib : p.attribs()) {
                    if (attrib.equalsIgnoreCase("id")) {
                        id = p.getAttributeValue().asLong();
                        got_id = true;
                    }
                }
                if (!got_id) continue;
                Way way = new Way(id);

                //Go through list of <nd> (node references) and <tag>
                if (p.enterTag()) {
                    for (StringRef tag2 : p.tags()) {
                        if (tag2.equalsIgnoreCase("nd")) {
                            //Ex: <nd ref="335923980"/>
                            for (StringRef att : p.attribs()) {
                                if (att.equalsIgnoreCase("ref")) {
                                    long ref = p.getAttributeValue().asLong();
                                    way.nodesLocalID.add(getNodeID(ref));
                                    break;
                                }
                            }
                        } else if (tag2.equalsIgnoreCase("tag")) {
                            //Ex: <tag k="highway" v="service"/>
                            String key = null;
                            String value = null;
                            for (StringRef att : p.attribs()) {
                                if (att.equalsIgnoreCase("k")) {
                                    key = p.getAttributeValue().getRawString();
                                } else if (att.equalsIgnoreCase("v")) {
                                    value = p.getAttributeValue().getRawString();
                                }
                            }
                            if (key != null && value != null) {
                                //way.tags.put(get_tag_id(key), value);
                                way.tags.put(getStringId(key), getStringId(value));
                            }
                        }
                    }
                }
                p.exitTag();

                addWay(way);
            } else if (tag.equalsIgnoreCase("bounds")) {
                //TODO Optional check uniqueness of bounds tag
                //Ex: <bounds minlat="50.7784900" minlon="6.0621200" maxlat="50.7815800" maxlon="6.0705400"/>

                //Check attributes from the <bounds> tag
                for (StringRef attrib : p.attribs()) {
                    if (attrib.equalsIgnoreCase("minlat")) {
                        min_corner.lat = p.getAttributeValue().asDouble();
                    } else if (attrib.equalsIgnoreCase("minlon")) {
                        min_corner.lon = p.getAttributeValue().asDouble();
                    } else if (attrib.equalsIgnoreCase("maxlat")) {
                        max_corner.lat = p.getAttributeValue().asDouble();
                    } else if (attrib.equalsIgnoreCase("maxlon")) {
                        max_corner.lon = p.getAttributeValue().asDouble();
                    }
                }
            }
        }


        //Sample code for tag and attribute iteration
        /* String tag2;
        do {
            tag2 = p.next_tag();
            if (tag2.equalsIgnoreCase("node")){
            } else if (tag2.equalsIgnoreCase("way")){
            } 
        }
        while (tag2.length() > 0); */

        /* String att;
        do {
            att = p.next_attribute();
            if (att.equalsIgnoreCase("minlat")){
                minlat = Double.parseDouble(p.get_attribute_value());
            } else if (attrib.equalsIgnoreCase("minlon")){
                minlon = Double.parseDouble(p.get_attribute_value());
            }  
        } while (att.length() > 0); */

        mid_point = min_corner.midpoint(max_corner);
        size = max_corner.subtract(min_corner);

        long end = System.nanoTime();
        System.out.println("Parsed OsmMap " + name + " in " + ((end - start) / 1000000.0) + " ms.");
    }


    public boolean isRoad(Way way) {
        int tagId = way.getTagId(TAG.HIGHWAY);
        return tagId >= 0 && roadTags.contains(tagId);
    }

    public boolean isBuilding(Way way) {
        return way.getTagId(TAG.BUILDING) >= 0;
    }

    private void getRoadTagsById() {
        roadTags.add(HIGHWAY.MOTORWAY.id);
        roadTags.add(HIGHWAY.TRUNK.id);
        roadTags.add(HIGHWAY.PRIMARY.id);
        roadTags.add(HIGHWAY.SECONDARY.id);
        roadTags.add(HIGHWAY.TERTIARY.id);
        roadTags.add(HIGHWAY.UNCLASSIFIED.id);
        roadTags.add(HIGHWAY.RESIDENTIAL.id);
        roadTags.add(HIGHWAY.MOTORWAY_LINK.id);
        roadTags.add(HIGHWAY.TRUNK_LINK.id);
        roadTags.add(HIGHWAY.PRIMARY_LINK.id);
        roadTags.add(HIGHWAY.SECONDARY_LINK.id);
        roadTags.add(HIGHWAY.TERTIARY_LINK.id);
        roadTags.add(HIGHWAY.LIVING_STREET.id);
        roadTags.add(HIGHWAY.SERVICE.id);
        roadTags.add(HIGHWAY.PEDESTRIAN.id);
        roadTags.add(HIGHWAY.TRACK.id);
        roadTags.add(HIGHWAY.RACEWAY.id);
    }

    @Override
    public String getKey() {
        return CONTEXT_KEY;
    }
}