package conflang._symboltable;

import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.ImportStatement;
import de.monticore.symboltable.MutableScope;

import java.util.List;
import java.util.Optional;

public class ConfLangArtifactScope extends ArtifactScope {

    public ConfLangArtifactScope(String packageName, List<ImportStatement> imports) {
        super(packageName, imports);
    }

    public ConfLangArtifactScope(Optional<MutableScope> enclosingScope, String packageName, List<ImportStatement> imports) {
        super(enclosingScope, packageName, imports);
    }

    public ConfigurationScope getConfLangConfigurationScope() {
        return null;
    }
}