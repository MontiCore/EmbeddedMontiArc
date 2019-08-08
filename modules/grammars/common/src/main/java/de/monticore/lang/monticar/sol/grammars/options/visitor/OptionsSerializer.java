/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.options.visitor;

import de.monticore.lang.monticar.sol.grammars.options._ast.ASTOption;
import org.json.JSONObject;

public interface OptionsSerializer {
    /**
     * Serializes a given option into a format understandable by the IDE.
     * @param node The AST node of the option to be serialized.
     * @return The serialized option.
     */
    JSONObject serialize(ASTOption node);
}
