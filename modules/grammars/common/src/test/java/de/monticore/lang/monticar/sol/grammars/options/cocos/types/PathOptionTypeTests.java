/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.options.cocos.types;

import de.monticore.lang.monticar.sol.grammars.options.cocos.types.props.LabelProp;
import de.monticore.lang.monticar.sol.grammars.options.cocos.types.props.Prop;
import de.monticore.lang.monticar.sol.grammars.options.cocos.types.props.RequiredProp;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

public class PathOptionTypeTests extends AbstractOptionTypeTests {
    LabelProp label = new LabelProp();
    RequiredProp required = new RequiredProp();
    PathOptionType type = new PathOptionType(label, required);

    @Override
    protected OptionType getType() {
        return type;
    }

    @Override
    protected Set<Prop> getExpectedProps() {
        return new HashSet<>(Arrays.asList(label, required));
    }

    @Override
    protected String getExpectedIdentifier() {
        return "path";
    }

    @Override
    protected boolean getExpectedAllowsSubOptions() {
        return false;
    }
}
