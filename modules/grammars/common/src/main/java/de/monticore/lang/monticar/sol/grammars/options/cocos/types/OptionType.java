/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.options.cocos.types;

import de.monticore.lang.monticar.sol.grammars.options.cocos.types.props.Prop;

import java.util.Set;

public interface OptionType { // TODO: Replace by simpler mechanism.
    /**
     * @return The identifier of the option to be used in the component factory.
     */
    String getIdentifier();

    /**
     * @return The supported props of this OptionType.
     */
    Set<Prop> getProps();

    /**
     * @return True if this OptionType supports nested options (such as lists), false otherwise.
     */
    default boolean allowsSubOptions() {
        return false;
    }
}
