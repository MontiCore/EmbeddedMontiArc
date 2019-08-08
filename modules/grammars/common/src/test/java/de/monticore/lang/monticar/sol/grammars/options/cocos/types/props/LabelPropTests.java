/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.options.cocos.types.props;

import de.monticore.lang.monticar.sol.grammars.options._ast.OptionsMill;
import de.monticore.mcliterals._ast.ASTStringLiteral;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class LabelPropTests {
    final LabelProp prop = new LabelProp();

    @Test
    void testGetIdentifier() {
        assertEquals("label", prop.getIdentifier(), "Identifier does not match.");
    }

    @Test
    void testAccepts() {
        ASTStringLiteral node = OptionsMill.stringLiteralBuilder().build();

        assertTrue(prop.accepts(node), "'label' should accept strings.");
    }
}
