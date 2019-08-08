/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.options.cocos.types.props;

import com.google.inject.Singleton;
import de.monticore.mcliterals._ast.ASTBooleanLiteral;

@Singleton
public class RequiredProp implements Prop {
    @Override
    public String getIdentifier() {
        return "required";
    }

    @Override
    public boolean accepts(ASTBooleanLiteral node) {
        return true;
    }
}
