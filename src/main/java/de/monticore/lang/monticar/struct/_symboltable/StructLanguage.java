/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.struct._symboltable;

import de.monticore.ast.ASTNode;
import de.monticore.modelloader.ModelingLanguageModelLoader;

public class StructLanguage extends StructLanguageTOP {

    public static final String FILE_ENDING = "struct";

    public StructLanguage() {
        super("C-Like Structures Definition Language", FILE_ENDING);
    }

    @Override
    protected ModelingLanguageModelLoader<? extends ASTNode> provideModelLoader() {
        return new StructModelLoader(this);
    }
}
