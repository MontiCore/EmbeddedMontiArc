/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponentInstance;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortArraySymbol;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberTypeArgument;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.symboltable.Symbol;
import de.monticore.types.types._ast.ASTSimpleReferenceType;
import de.monticore.types.types._ast.ASTTypeArgument;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 */
public class InstanceInformation {
    protected String compName;
    protected ASTSubComponent astSubComponent;

    public InstanceInformation() {

    }

    public InstanceInformation(String compName, ASTSubComponent astSubComponent) {
        this.compName = compName;
        this.astSubComponent = astSubComponent;
    }


    public String getCompName() {
        return compName;
    }

    public void setCompName(String compName) {
        this.compName = compName;
    }

    public ASTSubComponent getASTSubComponent() {
        return astSubComponent;
    }

    public void setASTSubComponent(ASTSubComponent astSubComponent) {
        this.astSubComponent = astSubComponent;
    }

    public int getInstanceNumberForArgumentIndex(int index) {
        return getInstanceNumberFromASTSubComponent(astSubComponent, index);
    }

    public int getInstanceNumberForPortName(String portName) {
        Symbol symbol = getASTSubComponent().getSymbolOpt().get();
        EMAComponentInstantiationSymbol emaComponentInstantiationSymbol = (EMAComponentInstantiationSymbol) symbol;
        Log.debug(emaComponentInstantiationSymbol.getComponentType().toString(), "EMAComponentInstantiationSymbol");
        Log.debug(portName, "PortName");
        EMAPortArraySymbol namedArray = emaComponentInstantiationSymbol.getComponentType().getPortArray(portName);
        if (namedArray != null && namedArray.getNameSizeDependsOn().isPresent())
            Log.debug(namedArray.getNameSizeDependsOn().get(), "PortArray Depends On:");

        int counter = 0;
        for (ResolutionDeclarationSymbol resolutionDeclarationSymbol : emaComponentInstantiationSymbol.getComponentType().getResolutionDeclarationSymbols()) {
            if (emaComponentInstantiationSymbol.getComponentType().isPortDependentOnResolutionDeclarationSymbol(portName, resolutionDeclarationSymbol.getNameToResolve())) {
                Log.debug("Name: " + portName + " nameToResolve: " + resolutionDeclarationSymbol.getNameToResolve(), "Porty Depends On:");
                return getInstanceNumberFromASTSubComponent(getASTSubComponent(), counter);
            }
            ++counter;
        }


        return -1;
    }


    public List<Integer> getInstanceNumberForArguments() {
        List<Integer> intList = new ArrayList<>();

        int curIndex = 0;
        int curResult = 0;
        while (true) {
            curResult = getInstanceNumberFromASTSubComponent(getASTSubComponent(), curIndex);
            if (curResult != -1) {
                intList.add(curResult);
            } else {
                break;
            }
        }

        return intList;
    }

    public static int getInstanceNumberFromASTSubComponent(ASTSubComponent subComponent, int index) {
        if (subComponent.getType() instanceof ASTSimpleReferenceType) {
            ASTSimpleReferenceType simpleReferenceType = (ASTSimpleReferenceType) subComponent.getType();
            return handleSimpleReferenceType(simpleReferenceType, index);
        }
        return -1;
    }

    private static int handleSimpleReferenceType(ASTSimpleReferenceType simpleReferenceType, int index) {
        if (simpleReferenceType.getTypeArgumentsOpt().isPresent()) {
            int counter = 0;
            for (ASTTypeArgument astTypeArgument : simpleReferenceType.getTypeArgumentsOpt().get().getTypeArgumentList()) {
                int result = handleSimpleReferenceType(astTypeArgument, index, counter);
                if (result != -1 && counter == index)
                    return result;
                else {
                    //Log.error("HERE");
                }
                ++counter;
            }
        } else {
            //Log.error("No arguments");
        }
        return -1;
    }

    public static int handleSimpleReferenceType(ASTTypeArgument astTypeArgument, int index, int counter) {
        int result = -1;
        if (astTypeArgument instanceof ASTUnitNumberTypeArgument) {
            if (((ASTUnitNumberTypeArgument) astTypeArgument).getNumberWithUnit().getNumber().isPresent()) {
                if (counter == index)
                    result = ((ASTUnitNumberTypeArgument) astTypeArgument).getNumberWithUnit().getNumber().get().intValue();
            }

        } else if (astTypeArgument instanceof ASTUnitNumberResolution) {
            if (((ASTUnitNumberResolution) astTypeArgument).getNumberWithUnitOpt().isPresent()) {
                if (counter == index)
                    result = ((ASTUnitNumberResolution) astTypeArgument).getNumber().get().intValue();
            }
        } else {
            //Log.error(astTypeArgument.getClass().toString());
        }
        return result;
    }

    public static String getInstanceNameFromASTSubComponent(ASTSubComponent subComponent, int index) {
        if (subComponent.getType() instanceof ASTSimpleReferenceType) {
            ASTSimpleReferenceType simpleReferenceType = (ASTSimpleReferenceType) subComponent.getType();
            if (simpleReferenceType.getTypeArgumentsOpt().isPresent()) {
                int counter = 0;
                for (ASTTypeArgument astTypeArgument : simpleReferenceType.getTypeArgumentsOpt().get().getTypeArgumentList()) {
                    if (astTypeArgument instanceof ASTUnitNumberResolution) {
                        if (((ASTUnitNumberResolution) astTypeArgument).getNameOpt().isPresent()) {
                            if (counter == index)
                                return ((ASTUnitNumberResolution) astTypeArgument).getNameOpt().get();
                            ++counter;
                        }

                    }
                }
            }

        }
        return null;
    }

    public static void setInstanceNumberInASTSubComponent(ASTSubComponent subComponent, String nameToSet, int numberToSet) {
        if (subComponent.getType() instanceof ASTSimpleReferenceType) {
            ASTSimpleReferenceType simpleReferenceType = (ASTSimpleReferenceType) subComponent.getType();
            if (simpleReferenceType.getTypeArgumentsOpt().isPresent()) {
                int counter = 0;
                for (ASTTypeArgument astTypeArgument : simpleReferenceType.getTypeArgumentsOpt().get().getTypeArgumentList()) {
                    if (astTypeArgument instanceof ASTUnitNumberResolution) {
                        if ((((ASTUnitNumberResolution) astTypeArgument).getNameOpt().isPresent())) {
                            String name = ((ASTUnitNumberResolution) astTypeArgument).getNameOpt().get();
                            if (name.equals(nameToSet))
                                ((ASTUnitNumberResolution) astTypeArgument).setNumber(Double.valueOf(numberToSet));
                            ++counter;
                        }

                    }
                }
            }

        }
    }

    @Override
    public String toString() {
        String subComponentString = "";
        for (ASTSubComponentInstance astSubComponentInstance : astSubComponent.getInstancesList()) {
            subComponentString += " " + astSubComponentInstance;
        }
        return "ComponentName: " + getCompName() + " " + subComponentString;
    }
}
