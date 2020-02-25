/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.commons.composite;

import java.util.HashMap;

/**
 * A BusComposite represents a class or structure with members. The contents map
 * should be filled with the member name and sub-component.
 */
public class BusComposite implements BusComponent {
    private HashMap<String, BusComponent> contents = new HashMap<>();

    public void setComponent(String key, BusComponent comp) {
        contents.put(key, comp);
    }

    public BusComponent getComponent(String key) {
        return contents.get(key);
    }

    HashMap<String, BusComponent> getContents() {
        return contents;
    }

    public ComponentType getType() {
        return ComponentType.COMPOSITE;
    }
}
