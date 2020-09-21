/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic;


import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceKind;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.cocos.EmbeddedMontiArcDynamicCoCos;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._ast.ASTEmbeddedMontiArcDynamicNode;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._cocos.EmbeddedMontiArcDynamicCoCoChecker;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._parser.EmbeddedMontiArcDynamicParser;

import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.EmbeddedMontiArcDynamicLanguage;

import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicComponentSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicEventHandlerSymbol;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import java.io.IOException;
import java.nio.file.*;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

import static org.junit.Assert.assertTrue;

public class ParserTest extends AbstractTest{

    @Test
    public void ParserTest_01_Parse() throws IOException {
        ParseTest parserTest = new ParseTest("." + "emad"/*EmbeddedMontiArcDynamicLanguage.FILE_ENDING*/);
        Files.walkFileTree(Paths.get("src/test/resources/test/embeddedmontiarcdynamic/parser"), parserTest);
        if (!parserTest.getErrors().isEmpty()) {
            Log.debug("Models in error", "ParserTest");
            for (String model : parserTest.getErrors()) {
                Log.debug("  " + model, "ParserTest");
            }
        }
        Log.info(
                "Count of tested models: " + parserTest.getTestCount(),
                "ParserTest"
        );
        Log.info(
                "Count of correctly parsed models: "
                        + (parserTest.getTestCount() - parserTest.getErrors().size()),
                "ParserTest"
        );
        assertTrue(
                "There were models that could not be parsed",
                parserTest.getErrors().isEmpty()
        );
    }

    @Test
    public void ParserTest_02_Fancy() throws IOException {
        ParseTest parserTest = new ParseTest("." + "emad"/*EmbeddedMontiArcDynamicLanguage.FILE_ENDING*/);
        Files.walkFileTree(Paths.get("src/test/resources/test/embeddedmontiarcdynamic/instanceFancy"), parserTest);
        if (!parserTest.getErrors().isEmpty()) {
            Log.debug("Models in error", "ParserTest");
            for (String model : parserTest.getErrors()) {
                Log.debug("  " + model, "ParserTest");
            }
        }
        Log.info(
                "Count of tested models: " + parserTest.getTestCount(),
                "ParserTest"
        );
        Log.info(
                "Count of correctly parsed models: "
                        + (parserTest.getTestCount() - parserTest.getErrors().size()),
                "ParserTest"
        );
        assertTrue(
                "There were models that could not be parsed",
                parserTest.getErrors().isEmpty()
        );
    }

    private static class ParseTest extends SimpleFileVisitor<Path> {

        private String fileEnding;
        private List<String> errors = new ArrayList<>();
        private int testCount = 0;

        public ParseTest(String fileEnding) {
            super();
            this.fileEnding = fileEnding;
        }

        /**
         * @return testCount
         */
        public int getTestCount() {
            return this.testCount;
        }

        /**
         * @return modelsInError
         */
        public List<String> getErrors() {
            return this.errors;
        }

        @Override
        public FileVisitResult visitFile(Path file, BasicFileAttributes attrs) throws IOException {
            Path par = file.getParent();
            if (file.toFile().isFile() && (file.toString().toLowerCase().endsWith(fileEnding))) {
                Log.debug("Parsing file " + file.toString(), "ParserTest");
                testCount++;

                EmbeddedMontiArcDynamicParser ep = new EmbeddedMontiArcDynamicParser();
                Optional<ASTEMACompilationUnit> parsedEvent = ep.parse(file.toString());
                if(ep.hasErrors() || !parsedEvent.isPresent()){
                    errors.add(file.toString());
                    Log.error("There were unexpected parser errors");
                }else{
                    Log.getFindings().clear();
                }


            }
            return FileVisitResult.CONTINUE;
        }
    }
}
