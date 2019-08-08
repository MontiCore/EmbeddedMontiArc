/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.options.cocos.types.props;

import de.monticore.lang.monticar.sol.grammars.options._ast.OptionsMill;
import de.monticore.mcliterals._ast.ASTBooleanLiteral;
import de.monticore.mcliterals._ast.ASTDoubleLiteral;
import de.monticore.mcliterals._ast.ASTStringLiteral;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class PropTests {
    @Mock Prop prop;

    @Test
    void testIsRequired() {
        when(prop.isRequired()).thenCallRealMethod();

        assertFalse(prop.isRequired(), "Prop should not be required by default.");
    }

    @Test
    void testAccepts() {
        ASTStringLiteral string = OptionsMill.stringLiteralBuilder().build();
        ASTDoubleLiteral number = OptionsMill.doubleLiteralBuilder().build();
        ASTBooleanLiteral bool = OptionsMill.booleanLiteralBuilder().build();

        assertFalse(prop.accepts(string), "Prop should not accept strings by default.");
        assertFalse(prop.accepts(number), "Prop should not accept doubles by default.");
        assertFalse(prop.accepts(bool), "Prop should not accept booleans by default.");
    }
}
