package conflang._symboltable;

import de.monticore.literals.mccommonliterals._ast.ASTSignedLiteral;

public class SimpleConfigurationEntrySymbol extends SimpleConfigurationEntrySymbolTOP {

    private ASTSignedLiteral value;

    public SimpleConfigurationEntrySymbol(String name) {
        super(name);
    }

    public ASTSignedLiteral getValue() {
        return value;
    }

    public void setValue(ASTSignedLiteral value) {
        this.value = value;
    }
}
