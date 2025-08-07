package de.monticore.lang.embeddedmontiarc.stream.lsp;

import de.monticore.ModelingLanguage;
import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.Utils;
import de.monticore.lang.monticar.enumlang._symboltable.EnumLangLanguage;
import de.monticore.lang.monticar.streamunits._ast.ASTStreamUnitsCompilationUnit;
import de.monticore.lang.monticar.streamunits._parser.StreamUnitsParser;
import de.monticore.lang.monticar.struct._ast.ASTStructCompilationUnit;
import de.monticore.lang.monticar.struct._ast.ASTStructNode;
import de.monticore.lang.monticar.struct._cocos.StructCoCoChecker;
import de.monticore.lang.monticar.struct._parser.StructParser;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.lang.monticar.struct._symboltable.StructSymbol;
import de.monticore.lang.monticar.struct.coco.DefaultStructCoCoChecker;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.SymbolKind;
import de.monticore.util.lsp.MontiCoreDocumentService;
import de.monticore.util.lsp.MontiCoreDocumentServiceWithSymbol;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.io.StringReader;
import java.nio.file.Path;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

public class StreamDocumentService extends MontiCoreDocumentService<ASTStreamUnitsCompilationUnit> {
    private StreamUnitsParser parser = new StreamUnitsParser();
    private ModelingLanguageFamily modelFamily;


    @Override
    protected String getLanguageServerIdentifier() {
        return "Stream Language Server";
    }

    @Override
    protected Optional<ASTStreamUnitsCompilationUnit> doParse(StringReader stringReader) {
        Log.info("Parsing!", "default");
        try {
            return parser.parse(stringReader);
        } catch (IOException e) {
            Log.error("Error parsing model: ", e);
        }
        return Optional.empty();
    }

    @Override
    protected Set<String> getModelFileExtensions() {
        return Collections.singleton("stream");
    }

    @Override
    protected void doCheckASTCocos(ASTStreamUnitsCompilationUnit astStreamUnitsCompilationUnit) {
        // No cocos implemented?
    }
}
