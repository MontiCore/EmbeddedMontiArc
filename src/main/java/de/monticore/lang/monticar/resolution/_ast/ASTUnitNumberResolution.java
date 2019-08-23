/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.resolution._ast;

import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.se_rwth.commons.logging.Log;

import javax.measure.unit.Unit;
import java.util.List;
import java.util.Optional;

/**
 * Created by Sascha on 01.06.2017.
 */
public class ASTUnitNumberResolution extends ASTUnitNumberResolutionTOP {
    public ASTUnitNumberResolution() {
        super();
    }

    public ASTUnitNumberResolution(Optional<String> name, Optional<ASTNumberWithUnit> unitNumber) {
        super(name, unitNumber);
    }

    public Optional<Double> getNumber() {
        return getNumberWithUnitOpt().get().getNumber();
    }

    public void setNumber(Double number) {
        if (!getNumberWithUnitOpt().isPresent()) {
            setNumberWithUnit(new ASTNumberWithUnit(Optional.empty(), Optional.empty(), Optional.empty()));
        }
        getNumberWithUnitOpt().get().setNumber(number);
    }

    public Unit getUnit() {
        return getNumberWithUnitOpt().get().getUnit();
    }

    public void setUnit(Unit unit) {
        if (!getNumberWithUnitOpt().isPresent()) {
            setNumberWithUnit(new ASTNumberWithUnit(Optional.empty(), Optional.empty(), Optional.empty()));
        }
        getNumberWithUnitOpt().get().setUnit(unit);
    }

    public String doResolution(List<ResolutionDeclarationSymbol> resolutionDeclarationSymbolList) {
        if (getNameOpt().isPresent()) {
            for (ResolutionDeclarationSymbol resDeclSym : resolutionDeclarationSymbolList) {

                if (resDeclSym.getNameToResolve().equals(getNameOpt().get())) {

                    Log.debug(resDeclSym.getASTResolution().toString(), "Found ResolutionDeclarationSymbol:");
                    ASTNumberWithUnit toSet = ((ASTUnitNumberResolution) resDeclSym.getASTResolution()).getNumberWithUnitOpt().get();


                    Log.debug("" + toSet.getNumber().get().intValue(), "ToSet Number:");

                    setNumber(toSet.getNumber().get());
                    setUnit(toSet.getUnit());
                    Log.debug("" + getNumber().get().intValue(), "PortResolution Number:");
                    Log.debug(getNameOpt().get(), "Name:");
                    return getNameOpt().get();
                }
            }

        }
        return null;
    }

    @Override
    public String printType() {
        return "UnitNumberResolution";
    }
}
