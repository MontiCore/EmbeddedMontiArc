package conflang._symboltable;

import conflang.GlobalConstants;
import de.monticore.io.paths.ModelPath;

public class ConfLangGlobalScope extends ConfLangGlobalScopeTOP {

    public ConfLangGlobalScope() {
        super();
    }

    public ConfLangGlobalScope(ModelPath modelPath) {
        super(modelPath, GlobalConstants.CONF_LANG_FILE_EXTENSION);
    }

    @Override
    public ConfLangGlobalScope getRealThis() {
        return this;
    }
}