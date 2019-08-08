/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.options.cocos.types;

import de.monticore.lang.monticar.sol.grammars.options.cocos.types.props.LabelProp;
import de.monticore.lang.monticar.sol.grammars.options.cocos.types.props.Prop;
import de.monticore.lang.monticar.sol.grammars.options.cocos.types.props.RequiredProp;

import java.util.*;

public abstract class AbstractOptionType implements OptionType {
    protected final Set<Prop> props;

    protected AbstractOptionType(LabelProp label, RequiredProp required, Prop ...props) {
        this.props = new HashSet<>();

        this.props.add(label);
        this.props.add(required);

        Collections.addAll(this.props, props);
    }

    @Override
    public Set<Prop> getProps() {
        return this.props;
    }
}
