package de.monticore.lang.gdl.cocos.types;

import de.monticore.lang.gdl._ast.ASTGDLNode;
import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._cocos.GDLCoCoChecker;

public class TypesCoCosChecker extends GDLCoCoChecker {
    
    private ValueTypesMustBeDefined defChecker = new ValueTypesMustBeDefined();

    public TypesCoCosChecker() {
        super();
        addCoCo(new RootTypeDefNoTokens());
        addCoCo(new TypeMapDefNoTokens());
        addCoCo(new TypesBoundOnce());
        addCoCo(new StateActionSpacesTyped());
        addCoCo(defChecker);
    }

    @Override
    public void checkAll(ASTGDLNode node) {
        if (node instanceof ASTGame) {
            defChecker.setASTGame((ASTGame) node);
        }
        super.checkAll(node);
    }

}
