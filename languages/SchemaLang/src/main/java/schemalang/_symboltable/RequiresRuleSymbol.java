package schemalang._symboltable;

import java.util.Collection;

public class RequiresRuleSymbol extends RequiresRuleSymbolTOP {

    private Collection<String> dependencies;

    public RequiresRuleSymbol(String name) {
        super(name);
    }

    public void setDependencies(Collection<String> dependencies) {
        this.dependencies = dependencies;
    }
}