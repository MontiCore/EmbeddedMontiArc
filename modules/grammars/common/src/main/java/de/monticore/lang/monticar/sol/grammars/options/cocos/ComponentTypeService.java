/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.options.cocos;

import java.util.Set;

/**
 * A service which can be used to query component information from a preconfigured file.
 */
public interface ComponentTypeService {
    /**
     * Checks whether a given component type has been registered.
     * @param componentType The component type to be checked.
     * @return True if the component type has been registered, false otherwise.
     */
    boolean isTypeRegistered(String componentType);

    /**
     * Checks whether a given component type supports sub-options.
     * @param componentType The component type to be checked.
     * @return True if the component type supports sub-options, false otherwise.
     */
    boolean supportsOptions(String componentType);

    /**
     * Checks whether a given component type supports a given prop.
     * @param componentType The component type to be checked.
     * @param prop The prop to be checked.
     * @return True if the component type supports the prop, false otherwise.
     */
    boolean supportsProp(String componentType, String prop);

    /**
     * Checks whether a given prop of a given component type supports a given type.
     * @param componentType The component type of the prop.
     * @param prop The prop to be checked.
     * @param type The type to be checked.
     * @return True if the prop supports the type, false otherwise.
     */
    boolean propSupportsType(String componentType, String prop, String type);

    /**
     * Returns all required props of a given component type.
     * @param componentType The component type from which the required props should be fetched.
     * @return A set of required props needed for the component type.
     */
    Set<String> getRequiredProps(String componentType);
}
