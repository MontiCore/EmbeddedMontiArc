/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.options.cocos.types;

import de.monticore.lang.monticar.sol.grammars.options.cocos.types.props.Prop;
import org.junit.jupiter.api.Test;

import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertIterableEquals;

public abstract class AbstractOptionTypeTests {
    protected abstract OptionType getType();

    protected abstract Set<Prop> getExpectedProps();

    protected abstract String getExpectedIdentifier();

    protected abstract boolean getExpectedAllowsSubOptions();

    @Test
    void testGetProps() {
        assertIterableEquals(
                this.getExpectedProps(),
                this.getType().getProps(),
                "Supported props do not match."
        );
    }

    @Test
    void testGetIdentifier() {
        assertEquals(
                this.getExpectedIdentifier(),
                this.getType().getIdentifier(),
                String.format("Identifier should be '%s'.", this.getType().getIdentifier())
        );
    }

    @Test
    void testAllowsSubOptions() {
        assertEquals(
                this.getExpectedAllowsSubOptions(),
                this.getType().allowsSubOptions(),
                String.format("'%s' should allow sub-options.", this.getType().getIdentifier())
        );
    }
}
