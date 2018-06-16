package de.monticore.lang.monticar.generator;

import de.ma2cfg.helper.Names;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.TypesPrinter;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

/**
 * @author Sascha Schneiders
 */
public class Helper {
    public static MathStatementsSymbol getMathStatementsSymbolFor(ExpandedComponentInstanceSymbol instanceSymbol, Scope symtab) {
        String resolveName = instanceSymbol.getPackageName() + "." + Names.FirstUpperCase(instanceSymbol.getName()) + ".MathStatements";
        MathStatementsSymbol mathSymbol = symtab.<MathStatementsSymbol>resolve(resolveName, MathStatementsSymbol.KIND).orElse(null);

        if (mathSymbol == null) {
            ComponentSymbol symbol = instanceSymbol.getComponentType().getReferencedSymbol();
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
