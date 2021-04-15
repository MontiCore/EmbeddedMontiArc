/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._parser;

import de.monticore.antlr4.MCConcreteParser;
import de.monticore.lang.math.ParserMathTest;
import org.junit.Ignore;

/**
 * Tests for MontiMathOpt's parser
 *
 */

public class ParserMathOptTest extends ParserMathTest {

    @Override
    protected MCConcreteParser getParser() {
        return new MathOptParser();
    }
}
