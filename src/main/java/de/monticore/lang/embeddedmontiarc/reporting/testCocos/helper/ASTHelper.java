package de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.LogConfig;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEmbeddedMontiArcNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathLanguage;
import de.monticore.lang.monticar.stream._symboltable.StreamLanguage;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.nio.file.Paths;

public class ASTHelper {
    public static ASTEmbeddedMontiArcNode getAstNode(String modelPath, String model, String fileType) throws CouldNotResolveException {
        try {
            Scope symTab = createSymTab(fileType, modelPath);
            ComponentSymbol comp = symTab.<ComponentSymbol>resolve(
                    model, ComponentSymbol.KIND).orElse(null);

            return (ASTEmbeddedMontiArcNode) comp.getAstNode().get();
        } catch (Exception e) {
            throw new CouldNotResolveException();
        }
    }

    public static Scope createSymTab(String fileType, String... modelPath) {
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        if ( fileType.equals("EMAM"))
            fam.addModelingLanguage(new EmbeddedMontiArcMathLanguage());
        else if ( fileType.equals("EMA"))
            fam.addModelingLanguage(new EmbeddedMontiArcLanguage());
        else
            Log.error("Unknown file type: " + fileType);
        fam.addModelingLanguage(new StreamLanguage());
        fam.addModelingLanguage(new StructLanguage());
        final ModelPath mp = new ModelPath();
        for (String m : modelPath) {
            mp.addEntry(Paths.get(m));
        }
        GlobalScope scope = new GlobalScope(mp, fam);

        de.monticore.lang.monticar.Utils.addBuiltInTypes(scope);
        LogConfig.init();
        return scope;
    }
}
