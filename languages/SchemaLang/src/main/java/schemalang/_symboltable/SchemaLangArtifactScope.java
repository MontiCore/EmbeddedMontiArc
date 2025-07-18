package schemalang._symboltable;

import de.monticore.symboltable.*;

import java.util.List;
import java.util.Optional;

public class SchemaLangArtifactScope extends ArtifactScope {

    public SchemaLangArtifactScope(String packageName, List<ImportStatement> imports) {
        super(packageName, imports);
    }

    public SchemaLangArtifactScope(Optional<MutableScope> enclosingScope, String packageName, List<ImportStatement> imports) {
        super(enclosingScope, packageName, imports);
    }

    public Optional<Symbol> resolveInHierarchy(String name, SymbolKind kind) {
        Optional<Symbol> resolvedSymbolOpt;
        for (MutableScope subScope : getSubScopes()) {
            resolvedSymbolOpt = subScope.resolve(name, kind);
            if (resolvedSymbolOpt.isPresent()) {
                return resolvedSymbolOpt;
            }
        }
        return Optional.empty();
    }
}
