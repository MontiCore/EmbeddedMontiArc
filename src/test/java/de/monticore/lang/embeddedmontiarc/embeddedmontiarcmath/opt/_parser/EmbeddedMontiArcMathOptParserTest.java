package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.opt._parser;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.ParserTest;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;

public class EmbeddedMontiArcMathOptParserTest extends ParserTest {

    @Override
    public void setUp() {
        super.setUp();
        setParser(new EmbeddedMontiArcMathParser());
    }
}