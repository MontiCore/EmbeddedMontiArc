/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.symboltable.Symbol;

public class CNNArchSymbolCoCo {

    public void check(Symbol sym){
        if (sym instanceof ArchitectureSymbol){
            check((ArchitectureSymbol) sym);
        }
        else if (sym instanceof LayerDeclarationSymbol){
            check((LayerDeclarationSymbol) sym);
        }
        else if (sym instanceof UnrollDeclarationSymbol){
            check((UnrollDeclarationSymbol) sym);
        }
        else if (sym instanceof UnrollInstructionSymbol){
            check((UnrollInstructionSymbol) sym);
        }
        else if (sym instanceof StreamInstructionSymbol){
            check((StreamInstructionSymbol) sym);
        }
        else if (sym instanceof ArchitectureElementSymbol){
            check((ArchitectureElementSymbol) sym);
        }
        else if (sym instanceof ArchExpressionSymbol){
            check((ArchExpressionSymbol) sym);
        }
        else if (sym instanceof ArchTypeSymbol){
            check((ArchTypeSymbol) sym);
        }
        else if (sym instanceof ParameterSymbol){
            check((ParameterSymbol) sym);
        }
        else if (sym instanceof ArgumentSymbol){
            check((ArgumentSymbol) sym);
        }
        else if (sym instanceof CNNArchCompilationUnitSymbol){
            check((CNNArchCompilationUnitSymbol) sym);
        }
        else if (sym instanceof VariableDeclarationSymbol){
            check((VariableDeclarationSymbol) sym);
        }
        else if (sym instanceof MathExpressionSymbol){
            check((MathExpressionSymbol) sym);
        }
        else{
            throw new IllegalStateException("Symbol class is unknown in CNNArchSymbolCoCo: "
                    + sym.getClass().getSimpleName());
        }
    }


    public void check(ArchitectureSymbol sym){
        //Override if needed
    }

    public void check(LayerDeclarationSymbol sym){
        //Override if needed
    }

    public void check(UnrollDeclarationSymbol sym){
        //Override if needed
    }

    public void check(ArchitectureElementSymbol sym){
        //Override if needed
    }

    public void check(ArchExpressionSymbol sym){
        //Override if needed
    }

    public void check(ArchTypeSymbol sym){
        //Override if needed
    }

    public void check(ParameterSymbol sym){
        //Override if needed
    }

    public void check(ArgumentSymbol sym){
        //Override if needed
    }

    public void check(CNNArchCompilationUnitSymbol sym){
        //Override if needed
    }

    public void check(VariableDeclarationSymbol sym){
        //Override if needed
    }

    public void check(MathExpressionSymbol sym){
        //Override if needed
    }

    public void check(UnrollInstructionSymbol sym){
        //Override if needed
    }

    public void check(StreamInstructionSymbol sym){
        //Override if needed
    }
}
