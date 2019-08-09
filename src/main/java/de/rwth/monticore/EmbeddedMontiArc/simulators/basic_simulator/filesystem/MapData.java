/**
 *
 * ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.filesystem;

import java.io.File;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.controller.SimpleCoordinateConverter;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Point2D;

public class MapData {

    public static class Way {
        //TODO only allocate tags if used
        public List<Long> nodes;
        //public HashMap<Integer, String> tags;
        public HashMap<String, String> tags;
        public boolean driveable;
        Way(){
            this.nodes = new LinkedList<>();
            this.tags = new HashMap<>();
            driveable = false;
        }
        public boolean has_tags(){
            return tags != null;
        }
    }

    public static class Node {
        //TODO only allocate tags if used
        public Point2D point;
        //public HashMap<Integer, String> tags;
        public HashMap<String, String> tags;
        Node(double lat, double lon){
            this.point = new Point2D(lat, lon);
            this.tags = new HashMap<>();
        }
    }

    public HashMap<Long, Node> nodes;
    public HashMap<Long, Way> ways;
    
    public double minlat;
    public double minlon;
    public double maxlat;
    public double maxlon;
    public Point2D min_corner;
    public Point2D max_corner;
    public Point2D size;

    public SimpleCoordinateConverter conv;

    /* //Maps an osm "tag" name (ex: "highway", "name", "maxspeed") to an id internal to this MapData object
    public HashMap<String, Integer> tag_ids;
    int tag_id_counter; */

    /* int get_tag_id(String tag_name){
        if (tag_ids.containsKey(tag_name))
            return tag_ids.get(tag_name);
        int id = tag_id_counter++;
        tag_ids.put(tag_name, id);
        return id;
    } */

    

    public void convertToApproximateMeters(){
        Point2D ref = new Point2D(minlat, minlon);
        conv = new SimpleCoordinateConverter(ref);
        for (HashMap.Entry<Long, Node> e : nodes.entrySet()){
            Node n = e.getValue();
            n.point = conv.latlonToMeters(n.point);
        }
        min_corner = conv.latlonToMeters(ref);
        max_corner = conv.latlonToMeters(new Point2D(maxlat, maxlon));
        size = max_corner.subtract(min_corner);
    }

    public void evaluate_driveable(){
        HashSet<String> allowed_ways = new HashSet<>();
        allowed_ways.add("motorway");
        allowed_ways.add("trunk");
        allowed_ways.add("primary");
        allowed_ways.add("secondary");
        allowed_ways.add("tertiary");
        allowed_ways.add("unclassified");
        allowed_ways.add("residential");
        allowed_ways.add("motorway_link");
        allowed_ways.add("trunk_link");
        allowed_ways.add("primary_link");
        allowed_ways.add("secondary_link");
        allowed_ways.add("tertiary_link");
        allowed_ways.add("living_street");
        allowed_ways.add("service");
        allowed_ways.add("pedestrian");
        allowed_ways.add("track");
        allowed_ways.add("raceway");
        for (Map.Entry<Long, Way> c : ways.entrySet()){
            MapData.Way way = c.getValue();
            if (way.has_tags()){
                String val = way.tags.get("highway");
                if (val != null){
                    if (allowed_ways.contains(val)){
                        way.driveable = true;
                    }
                }
            }
        }
    }

    public MapData(String name, FileSystem fileSystem) throws Exception {
        long start = System.nanoTime();
        File file = fileSystem.getPath("maps", name + ".osm");
        

        minlat = 0;
        minlon = 0;
        maxlat = 0;
        maxlon = 0;
        //tag_id_counter = 0;
        //tag_ids = new HashMap<>();
        nodes = new HashMap<>();
        ways = new HashMap<>();
        

        XmlTraverser p = new XmlTraverser();
        p.from_file(file);

        //Find osm tag
        String tag;
        do {
            tag = p.next_tag();
        }
        while (tag.length() > 0 && !tag.equalsIgnoreCase("osm"));

        if (tag.length() == 0) throw new Exception("Could not find <osm> tag");

        //Ex: <osm version="0.6" generator="CGImap 0.7.5 (14605 thorn-01.openstreetmap.org)" copyright="OpenStreetMap and contributors" attribution="http://www.openstreetmap.org/copyright" license="http://opendatacommons.org/licenses/odbl/1-0/">
        
        //Search the "version" attribute from the <osm> tag.
        String attrib;
        do {
            attrib = p.next_attribute();
        }
        while (attrib.length() > 0 && !attrib.equalsIgnoreCase("version"));

        //Validate version
        if (attrib.length() == 0) throw new Exception("Could not find osm version attribute");
        String version = p.get_attribute_value();
        if (!version.equalsIgnoreCase("0.6")) throw new Exception("Unsupported osm version: " + version);

        //Enter <osm> tag
        p.enter_tag();
        //Check all tags from the <osm> tag.
        do {
            tag = p.next_tag();
            if (tag.equalsIgnoreCase("node")){
                //Ex: <node id="6152869602" visible="true" version="1" changeset="65685715" timestamp="2018-12-22T05:39:15Z" user="adjuva" uid="92274" lat="50.7802316" lon="6.0716610"/>
                long id = 0;
                boolean got_id = false;
                double lat = 0;
                double lon = 0;
                do {
                    attrib = p.next_attribute();
                    if (attrib.equalsIgnoreCase("id")){
                        got_id = true;
                        id = Long.parseLong(p.get_attribute_value());
                    } else if (attrib.equalsIgnoreCase("lat")){
                        lat = Double.parseDouble(p.get_attribute_value());
                    } else if (attrib.equalsIgnoreCase("lon")){
                        lon = Double.parseDouble(p.get_attribute_value());
                    } 
                } while (attrib.length() > 0);
                if (!got_id) continue;
                Node node = new Node(lat, lon);

                //Check node tags
                if (p.enter_tag()){
                    String tag2;
                    do {
                        tag2 = p.next_tag();
                        if (tag2.equalsIgnoreCase("tag")){
                            //Ex: <tag k="railway" v="signal"/>
                            String att;
                            String key = null;
                            String value = null;
                            do {
                                att = p.next_attribute();
                                if (att.equalsIgnoreCase("k")){
                                    key = p.get_attribute_value();
                                } else if (att.equalsIgnoreCase("v")){
                                    value = p.get_attribute_value();
                                }  
                            } while (att.length() > 0);
                            if (key != null && value != null){
                                //node.tags.put(get_tag_id(key), value);
                                node.tags.put(key, value);
                            }
                        } 
                    }
                    while (tag2.length() > 0);
                }
                p.exit_tag();
                


                nodes.put(id, node);
            } else if (tag.equalsIgnoreCase("way")){
                //Ex: <way id="5168924" visible="true" version="23" changeset="48959749" timestamp="2017-05-24T23:00:23Z" user="Steffen van Bergerem" uid="61868">
                long id = 0;
                boolean got_id = false;
                do {
                    attrib = p.next_attribute();
                    if (attrib.equalsIgnoreCase("id")){
                        id = Long.parseLong(p.get_attribute_value());
                        got_id = true;
                    }
                } while (attrib.length() > 0);
                if (!got_id) continue;
                Way way = new Way();

                //Go through list of <nd> (node references) and <tag>
                if (p.enter_tag()){
                    String tag2;
                    do {
                        tag2 = p.next_tag();
                        if (tag2.equalsIgnoreCase("nd")){
                            //Ex: <nd ref="335923980"/>
                            String att;
                            do {
                                att = p.next_attribute();
                            }
                            while (att.length() > 0 && !att.equalsIgnoreCase("ref"));
                            if (att.length() == 0) return;
                            Long ref_id = Long.parseLong(p.get_attribute_value());
                            way.nodes.add(ref_id);
                        } else if (tag2.equalsIgnoreCase("tag")){
                            //Ex: <tag k="highway" v="service"/>
                            String att;
                            String key = null;
                            String value = null;
                            do {
                                att = p.next_attribute();
                                if (att.equalsIgnoreCase("k")){
                                    key = p.get_attribute_value();
                                } else if (att.equalsIgnoreCase("v")){
                                    value = p.get_attribute_value();
                                }  
                            } while (att.length() > 0);
                            if (key != null && value != null){
                                //way.tags.put(get_tag_id(key), value);
                                way.tags.put(key, value);
                            }
                        } 
                    }
                    while (tag2.length() > 0);
    
                }
                p.exit_tag();
                
                ways.put(id, way);
            } else if (tag.equalsIgnoreCase("bounds")){
                //TODO Optional check uniqueness of bounds tag
                //Ex: <bounds minlat="50.7784900" minlon="6.0621200" maxlat="50.7815800" maxlon="6.0705400"/>
                
                //Check attributes from the <bounds> tag
                do {
                    attrib = p.next_attribute();
                    if (attrib.equalsIgnoreCase("minlat")){
                        minlat = Double.parseDouble(p.get_attribute_value());
                    } else if (attrib.equalsIgnoreCase("minlon")){
                        minlon = Double.parseDouble(p.get_attribute_value());
                    } else if (attrib.equalsIgnoreCase("maxlat")){
                        maxlat = Double.parseDouble(p.get_attribute_value());
                    } else if (attrib.equalsIgnoreCase("maxlon")){
                        maxlon = Double.parseDouble(p.get_attribute_value());
                    } 
                } while (attrib.length() > 0);

            } 
        }
        while (tag.length() > 0);


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



        convertToApproximateMeters();
        evaluate_driveable();
        long end = System.nanoTime();
        System.out.println("Loaded map " + name + " in " + ((end-start)/1000000.0) + " ms.");
    }

}