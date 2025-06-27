package conflang._symboltable;

import conflang.GlobalConstants;
import de.monticore.ast.ASTNode;
import de.monticore.modelloader.ModelingLanguageModelLoader;

public class ConfLangLanguage extends ConfLangLanguageTOP {

    public ConfLangLanguage() {
        super("ConfLang", GlobalConstants.CONF_LANG_FILE_EXTENSION);
    }

    @Override
    protected ModelingLanguageModelLoader<? extends ASTNode> provideModelLoader() {
        return new ConfLangModelLoader(this);
    }

    @Override
    protected void initResolvingFilters() {
        super.initResolvingFilters();
        setModelNameCalculator(new ConfLangModelNameCalculator());
    }
}
