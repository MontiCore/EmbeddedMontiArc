package de.monticore.lang.embeddedmontiarc.embeddedmontiarcdl.lsp;

import de.monticore.ModelingLanguage;
import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.helper.ConstantPortHelper;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.EventLanguage;
import de.monticore.lang.monticar.emadl._cocos.EMADLCocos;
import de.monticore.lang.monticar.emadl._parser.EMADLParser;
import de.monticore.lang.monticar.emadl._symboltable.EMADLLanguage;
import de.monticore.lang.monticar.enumlang._symboltable.EnumLangLanguage;
import de.monticore.lang.monticar.streamunits._symboltable.StreamUnitsLanguage;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.SymbolKind;
import de.monticore.util.lsp.ModelFileCache;
import de.monticore.util.lsp.MontiCoreDocumentServiceWithSymbol;
import de.monticore.util.lsp.features.completion.LookaheadProvider;
import de.se_rwth.commons.logging.Log;
import org.jetbrains.annotations.NotNull;

import java.io.IOException;
import java.io.StringReader;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

public class EmadlDocumentService extends MontiCoreDocumentServiceWithSymbol<ASTEMACompilationUnit, EMAComponentSymbol> {
    private EMADLParser parser = new EMADLParser();
    private ModelingLanguageFamily modelFamily;

    public EmadlDocumentService(@NotNull ModelFileCache modelFileCache) {
        super(modelFileCache);
    }

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
    protected LookaheadProvider getLookaheadProvider() {
        return null;
    }

    @Override
    protected String getSymbolName(ASTEMACompilationUnit node) {
        return node.getComponent().getName();
    }

    @Override
    protected String getFullSymbolName(ASTEMACompilationUnit node) {
        return String.join(".",getPackageList(node)) + "." + getSymbolName(node);
    }

    protected EMAComponentInstanceSymbol getInstanceSymbol(EMAComponentSymbol sym) {
        return (EMAComponentInstanceSymbol) sym.getEnclosingScope()
        .resolveLocally(EMAComponentInstanceSymbol.KIND)
        .stream().findFirst().get();
    }

    @Override
    protected void doCheckSymbolCoCos(Path sourcePath, EMAComponentSymbol sym) {
        EMADLCocos checker = new EMADLCocos();
        EMAComponentInstanceSymbol instancesym = getInstanceSymbol(sym);

        checker.checkAll(instancesym);
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

    // TODO add all relevant languages
    protected ModelingLanguageFamily getModelingLanguageFamily() {
        if(modelFamily == null) {
            modelFamily = new ModelingLanguageFamily();
            modelFamily.addModelingLanguage(new EMADLLanguage());
            modelFamily.addModelingLanguage(new StreamUnitsLanguage());
            modelFamily.addModelingLanguage(new StructLanguage());
            modelFamily.addModelingLanguage(new EnumLangLanguage());
            modelFamily.addModelingLanguage(new EventLanguage());
        }

        return modelFamily;
    }
}
