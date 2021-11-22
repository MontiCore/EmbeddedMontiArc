package schemalang._ast;

import de.monticore.mcbasictypes1._ast.ASTQualifiedName;
import schemalang._symboltable.ReferenceModelSymbol;

import java.util.List;
import java.util.Optional;

public class ASTReferenceModel extends ASTReferenceModelTOP {

    public ASTReferenceModel() {
    }

    public ASTReferenceModel(List<ASTQualifiedName> referenceModels, String name) {
        super(referenceModels, name);
    }

    @Override
    public Optional<ReferenceModelSymbol> getReferenceModelSymbolOpt() {
        if (symbol.isPresent()) {
            ReferenceModelSymbol referenceModelSymbol = (ReferenceModelSymbol) symbol.get();
            return Optional.of(referenceModelSymbol);
        }
        return super.getReferenceModelSymbolOpt();
    }
}