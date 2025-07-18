/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils;

import java.util.HashMap;
import java.util.Optional;

/**
 * An agnostic registry for objects needed when building other objects.
 * Ex: Json Deserialize, Building EEComponents, Building Vehicles, ...
 * The BuildContext can be built on a previous BuildContext (recursively).
 */
public class BuildContext {
    final BuildContext parent;
    final HashMap<String, Object> map = new HashMap<>();
    public BuildContext() {
        this.parent = null;
    }
    public BuildContext(BuildContext parent) {
        this.parent = parent;
    }

    /** Adds a 'BuildObject' to the context, which will always have the same key. */
    public void addObject(BuildObject obj) {
        String key = obj.getKey();
        if (map.containsKey(key)) throw new IllegalArgumentException("Adding key '"+key+"' multiple times to the BuildContext.");
        map.put(key, obj);
    }
    /** Adds an object to the context under the given key. */
    public void addObject(Object obj, String key) {
        if (map.containsKey(key)) throw new IllegalArgumentException("Adding key '"+key+"' multiple times to the BuildContext.");
        map.put(key, obj);
    }

    /** 
     * Returns the given object from the build context or its parents. 
     * Guarantees non-null (throws exception else).
     * Might throw casting exception if wrong type.
     */
    public <T> T getObject(String key) {
        Object obj = map.get(key);
        if (obj != null) return (T) obj;
        if (parent != null) return parent.getObject(key);
        throw new IllegalArgumentException("Missing entry '"+key+"' in the BuildContext.");
    }

    public <T> Optional<T> getOptionalObject(String key) {
        Object obj = map.get(key);
        if (obj != null) return Optional.of((T) obj);
        if (parent != null) return parent.getObject(key);
        return Optional.empty();
    }
}
