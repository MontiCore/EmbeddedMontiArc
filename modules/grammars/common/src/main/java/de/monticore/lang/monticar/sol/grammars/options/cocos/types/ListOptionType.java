/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.options.cocos.types;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.options.cocos.types.props.LabelProp;
import de.monticore.lang.monticar.sol.grammars.options.cocos.types.props.RequiredProp;

@Singleton
public class ListOptionType extends AbstractOptionType {
    @Inject
    protected ListOptionType(LabelProp label, RequiredProp required) {
        super(label, required);
    }

    @Override
    public String getIdentifier() {
        return "list";
    }

    @Override
    public boolean allowsSubOptions() {
        return true;
    }
}
