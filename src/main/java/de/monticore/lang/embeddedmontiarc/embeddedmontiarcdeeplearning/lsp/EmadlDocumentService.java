package de.monticore.lang.embeddedmontiarc.embeddedmontiarcdeeplearning.lsp;

import de.monticore.ModelingLanguage;
import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;

/* TODO add EMADL to dependencies @ //deeplearning */
//deeplearning
import de.monticore.lang.embeddedmontiarc.cocos.EmbeddedMontiArcCoCos;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcCoCoChecker;
//deeplearning
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;

// import de.monticore.lang.monticar.cnnarch._cocos.CNNArchCocos;
import de.monticore.lang.monticar.emadl._cocos.EMADLCocos;
//deeplearning
import de.monticore.lang.monticar.emadl._parser.EMADLParser;
import de.monticore.lang.monticar.emadl._symboltable.EMADLLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.embeddedmontiarc.helper.ConstantPortHelper;
//other
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.EventLanguage;
//other
import de.monticore.lang.monticar.enumlang._symboltable.EnumLangLanguage;
//other
import de.monticore.lang.monticar.streamunits._symboltable.StreamUnitsLanguage;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.SymbolKind;
import de.monticore.util.lsp.MontiCoreDocumentServiceWithSymbol;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.io.StringReader;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

public class EmadlDocumentService extends MontiCoreDocumentServiceWithSymbol<ASTEMACompilationUnit, EMAComponentSymbol> {
    // private EmbeddedMontiArcDLParser parser = new EmbeddedMontiArcDLParser();
    private EMADLParser parser = new EMADLParser();
    private ModelingLanguageFamily modelFamily;

    @Override
    public String getLanguageServerIdentifier() {
        return "EMADL Parser";
    }

    @Override
    public Optional<ASTEMACompilationUnit> doParse(StringReader fullText) {
        Log.info("Parsing!", "default");
        try {
            return parser.parse(fullText);
        } catch (IOException e) {
            Log.error("Error parsing model: ", e);
        }
        return Optional.empty();
    }

    @Override
    protected List<String> getPackageList(ASTEMACompilationUnit node) {
        return node.getPackageList();
    }

    @Override
    protected String getSymbolName(ASTEMACompilationUnit node) {
        return node.getComponent().getName();
    }

    @Override
    protected String getFullSymbolName(ASTEMACompilationUnit node) {
        return String.join(".",getPackageList(node)) + "." + getSymbolName(node);
    }

    @Override
    protected void doCheckSymbolCoCos(Path sourcePath, EMAComponentSymbol sym) {
        // TODO
        // EmbeddedMontiArcCoCoChecker checker = EmbeddedMontiArcCoCos.createChecker();
        EMADLCocos checker = new EMADLCocos();
        checker.checkAll((EMAComponentInstanceSymbol) sym.getAstNode().get());
        if (de.monticore.lang.math.LogConfig.getFindings().isEmpty()) {
            Log.info("No CoCos invalid", "default");
        } else {
            Log.info("Findings: " + de.monticore.lang.math.LogConfig.getFindings(), "default");
        }
    }

    @Override
    protected Set<String> getModelFileExtensions() {
        return getModelingLanguageFamily().getModelingLanguages().stream().map(ModelingLanguage::getFileExtension).collect(Collectors.toSet());
    }

    @Override
    protected SymbolKind getSymbolKind() {
        return EMAComponentSymbol.KIND;
    }

    @Override
    public Scope createSymTab(Path... modelPath) {
        ConstantPortHelper.resetLastID();

        ModelingLanguageFamily fam = getModelingLanguageFamily();
        final ModelPath mp = new ModelPath();
        for (Path m : modelPath) {
            mp.addEntry(m);
        }

        GlobalScope scope = new GlobalScope(mp, fam);
        de.monticore.lang.monticar.Utils.addBuiltInTypes(scope);
        return scope;
    }

    protected ModelingLanguageFamily getModelingLanguageFamily() {
        if(modelFamily == null) {
            modelFamily = new ModelingLanguageFamily();
            // TODO
            // EmbeddedMontiArcMathLanguage montiArcMathLanguage = new EmbeddedMontiArcMathLanguage();
            // modelFamily.addModelingLanguage(montiArcMathLanguage);
            EMADLLanguage montiArcEMADLLanguage = new EMADLLanguage();
            modelFamily.addModelingLanguage(montiArcEMADLLanguage);
            modelFamily.addModelingLanguage(new StreamUnitsLanguage());
            modelFamily.addModelingLanguage(new StructLanguage());
            modelFamily.addModelingLanguage(new EnumLangLanguage());
            modelFamily.addModelingLanguage(new EventLanguage());
        }

        return modelFamily;
    }
}
