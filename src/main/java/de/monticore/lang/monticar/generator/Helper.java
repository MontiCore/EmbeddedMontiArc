/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.ma2cfg.helper.Names;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

/**
 * @author Sascha Schneiders
 */
public class Helper {
    public static MathStatementsSymbol getMathStatementsSymbolFor(EMAComponentInstanceSymbol instanceSymbol, Scope symtab) {
        String resolveName = instanceSymbol.getPackageName() + "." + Names.FirstUpperCase(instanceSymbol.getName()) + ".MathStatements";
        MathStatementsSymbol mathSymbol = symtab.<MathStatementsSymbol>resolve(resolveName, MathStatementsSymbol.KIND).orElse(null);

        if (mathSymbol == null) {
            EMAComponentSymbol symbol = instanceSymbol.getComponentType().getReferencedSymbol();
            resolveName = symbol.getPackageName() + "." + symbol.getName() + ".MathStatements";
            mathSymbol = symtab.<MathStatementsSymbol>resolve(resolveName, MathStatementsSymbol.KIND).orElse(null);
        }

        if (mathSymbol != null)
            Log.info(mathSymbol.toString(), "MathSymbol:");
        else
            Log.info("Could not resolve " + resolveName, "MathSymbol:");
        return mathSymbol;
    }
}
