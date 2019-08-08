/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.options.cocos.types.props;

import com.google.inject.Singleton;
import de.monticore.mcliterals._ast.ASTStringLiteral;

@Singleton
public class LabelProp implements Prop {
    @Override
    public String getIdentifier() {
        return "label";
    }

    @Override
    public boolean isRequired() {
        return true;
    }

    @Override
    public boolean accepts(ASTStringLiteral node) {
        return true;
    }
}
