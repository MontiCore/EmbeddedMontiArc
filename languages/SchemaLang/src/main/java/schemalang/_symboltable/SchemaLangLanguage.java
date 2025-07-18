package schemalang._symboltable;

import de.monticore.ast.ASTNode;
import de.monticore.modelloader.ModelingLanguageModelLoader;
import schemalang.GlobalConstants;

public class SchemaLangLanguage extends schemalang._symboltable.SchemaLangLanguageTOP {

    public SchemaLangLanguage() {
        super("SchemaLang", GlobalConstants.SCHEMA_LANG_FILE_EXTENSION);
    }

    @Override
    protected ModelingLanguageModelLoader<? extends ASTNode> provideModelLoader() {
        return new SchemaLangModelLoader(this);
    }

    @Override
    protected void initResolvingFilters() {
        super.initResolvingFilters();
        setModelNameCalculator(new SchemaLangModelNameCalculator());
    }
}