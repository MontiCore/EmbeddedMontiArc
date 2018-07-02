/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponentInstance;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortArraySymbol;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberTypeArgument;
import de.monticore.symboltable.Symbol;
import de.monticore.types.types._ast.ASTSimpleReferenceType;
import de.monticore.types.types._ast.ASTTypeArgument;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Sascha Schneiders
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