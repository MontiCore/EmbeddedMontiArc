package de.monticar.lang.monticar.generator.python;

import java.util.ArrayList;

//TODO: replace with dependency: EMAM2PythonRos!
//TODO: rename: not used as a Tag
public class RosTag {
    public String component = "";
    public ArrayList<RosInterface> subscriber = new ArrayList<>();
    public ArrayList<RosInterface> publisher = new ArrayList<>();
}
