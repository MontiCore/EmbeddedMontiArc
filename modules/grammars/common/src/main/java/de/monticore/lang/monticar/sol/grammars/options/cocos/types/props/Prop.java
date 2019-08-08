/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.options.cocos.types.props;

import de.monticore.mcliterals._ast.ASTBooleanLiteral;
import de.monticore.mcliterals._ast.ASTDoubleLiteral;
import de.monticore.mcliterals._ast.ASTStringLiteral;

public interface Prop { // TODO: Replace by simpler mechanism.
    /**
     * @return The identifier of the prop to be set.
     */
    String getIdentifier();

    /**
     * @return True if the prop is required, false otherwise.
     */
    default boolean isRequired() {
        return false;
    }

    /**
     * @param node The node type to be checked against.
     * @return True if the prop accepts strings as value type, false otherwise.
     */
    default boolean accepts(ASTStringLiteral node) {
        return false;
    }

    /**
     * @param node The node type to be checked against.
     * @return True if the prop accepts doubles as value type, false otherwise.
     */
    default boolean accepts(ASTDoubleLiteral node) {
        return false;
    }

    /**
     * @param node The node type to be checked against.
     * @return True if the prop accepts booleans as value type, false otherwise.
     */
    default boolean accepts(ASTBooleanLiteral node) {
        return false;
    }
}
