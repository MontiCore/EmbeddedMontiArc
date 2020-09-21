/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic;

import de.monticore.lang.embeddedmontiarc.cocos.EmbeddedMontiArcCoCos;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEmbeddedMontiArcNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.cocos.EmbeddedMontiArcDynamicCoCos;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._parser.EmbeddedMontiArcDynamicParser;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;
import org.apache.commons.io.filefilter.DirectoryFileFilter;
import org.apache.commons.io.filefilter.RegexFileFilter;
import org.junit.Ignore;
import org.junit.Test;

import java.io.IOException;
import java.nio.file.*;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class CocoTest extends AbstractTest {

    @Test
    public void CocoTest_01_Valid() throws IOException {
        checkValid("valid", "componentDynamic.V1");
        checkValid("valid", "componentDynamic.V2");
        checkValid("valid", "componentDynamic.V3");
        checkValid("valid", "newDynamicComponentAndPort.V1");
        checkValid("valid", "newInOutSideEvent.V1");
    }

    @Test
    public void CocoTest_02_InValid() throws IOException {

        checkValid("invalid", "componentDynamic.V1", 2, "xAD002");
        checkValid("invalid", "newDynamicComponentAndPort.V1", 1, "xAD004");
        checkValid("invalid", "newDynamicComponentAndPort.V2", 1, "xAD005");
        checkValid("invalid", "newDynamicComponentAndPort.V3", 2, "xAD005", "xCC003");
        checkValid("invalid", "newInOutSideEvent.V1", 1, "xAD003");
    }

    private static class TestVisitor extends SimpleFileVisitor<Path> {

        private String fileEnding;
        private List<String> errors = new ArrayList<>();
        private int testCount = 0;
        private boolean expectError = false;

        public TestVisitor(String fileEnding, boolean expError) {
            super();
            this.fileEnding = fileEnding;
            this.expectError = expError;
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
            Log.getFindings().clear();
            if (file.toFile().isFile() && (file.toString().toLowerCase().endsWith(fileEnding))) {
                Log.debug("Parsing file " + file.toString(), "ParserTest");
                testCount++;

                EmbeddedMontiArcDynamicParser ep = new EmbeddedMontiArcDynamicParser();
                Optional<ASTEMACompilationUnit> parsedEvent = ep.parse(file.toString());

                if (ep.hasErrors() || !parsedEvent.isPresent()) {
                    errors.add(file.toString());
                    Log.error("There were unexpected parser errors");
                } else {
                    Log.getFindings().clear();
                }

                EmbeddedMontiArcDynamicCoCos.createChecker().checkAll(parsedEvent.get());
                if (!this.expectError) {
                    if (Log.getFindings().size() > 0) {
                        errors.add(file.toString());
                        Log.error("There were unexpected coco errors");
                    }
                } else {
                    if (Log.getFindings().size() == 0) {
                        errors.add(file.toString());
                        Log.error("There were NO expected coco errors");
                    }
                }
            }
            return FileVisitResult.CONTINUE;
        }
    }


}
