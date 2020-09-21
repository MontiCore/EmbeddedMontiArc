/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.enumlang._symboltable;

import de.monticore.ast.ASTNode;
import de.monticore.modelloader.ModelingLanguageModelLoader;

public class EnumLangLanguage extends EnumLangLanguageTOP {

    public static final String FILE_ENDING = "enum";

    public EnumLangLanguage() {
        super("Enum Definition Language", FILE_ENDING);
    }

    @Override
    protected ModelingLanguageModelLoader<? extends ASTNode> provideModelLoader() {
        return new EnumLangModelLoader(this);
    }
}
