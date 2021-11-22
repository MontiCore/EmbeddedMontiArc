package schemalang._ast;

import com.google.common.base.Joiner;
import schemalang.SchemaMemberType;
import schemalang._symboltable.RequiresRuleSymbol;
import schemalang._symboltable.TypedDeclarationSymbol;

import java.util.List;
import java.util.Optional;

public class ASTRequiresRule extends ASTRequiresRuleTOP {

    public ASTRequiresRule() {
    }

    public ASTRequiresRule(List<String> memberss, String name) {
        super(memberss, name);
    }

    @Override
    public Optional<RequiresRuleSymbol> getRequiresRuleSymbolOpt() {
        if (symbol.isPresent()) {
            RequiresRuleSymbol requiresRuleSymbol = (RequiresRuleSymbol) symbol.get();
            return Optional.of(requiresRuleSymbol);
        }
        return super.getRequiresRuleSymbolOpt();
    }

    @Override
    public SchemaMemberType getSchemaMemberType() {
        return SchemaMemberType.REQUIRES_RULE;
    }

    @Override
    public String toString() {
        return getName().concat(" requires ").concat(Joiner.on(", ").join(dependenciess));
    }
}