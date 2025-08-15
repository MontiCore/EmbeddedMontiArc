/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.monticore.lang.monticar.cnnarch.helper.Utils;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class CNNArchCompilationUnitSymbol extends CNNArchCompilationUnitSymbolTOP{

    private List<ParameterSymbol> parameters;
    private List<IODeclarationSymbol> ioDeclarations;
    private ArchitectureSymbol architecture;

    public CNNArchCompilationUnitSymbol(String name) {
        super(name);
    }

    public ArchitectureSymbol getArchitecture() {
        return architecture;
    }

    public void setArchitecture(ArchitectureSymbol architecture) {
        this.architecture = architecture;
    }

    public List<IODeclarationSymbol> getIoDeclarations() {
        return ioDeclarations;
    }

    public void setIoDeclarations(List<IODeclarationSymbol> ioDeclarations) {
        this.ioDeclarations = ioDeclarations;
    }

    public List<ParameterSymbol> getParameters() {
        return parameters;
    }

    public void setParameters(List<ParameterSymbol> parameters) {
        this.parameters = parameters;
    }

    public ArchitectureSymbol resolve(){
        checkParameters();
        getArchitecture().resolve();
        return getArchitecture();
    }


    public void checkParameters(){
        for (ParameterSymbol parameter : getParameters()){
            if (!parameter.hasExpression()){
                Log.error("0" + ErrorCodes.MISSING_VAR_VALUE + " Missing architecture argument. " +
                        "The parameter '" + parameter.getName() + "' has no value.");
            }
        }
    }

    public Optional<ParameterSymbol> getParameter(String name){
        for (ParameterSymbol parameter : getParameters()){
            if (parameter.getName().equals(name)){
                return Optional.of(parameter);
            }
        }
        return Optional.empty();
    }

    private ParameterSymbol getParameterOrError(String name){
        Optional<ParameterSymbol> param = getParameter(name);
        if (param.isPresent()){
            return param.get();
        }
        else {
            throw new IllegalArgumentException("architecture parameter with name " + name + " does not exist.");
        }
    }

    public void setParameter(String name, Rational value){
        ParameterSymbol parameter = getParameterOrError(name);
        if (value.getDivisor().intValue() == 1){
            parameter.setExpression(ArchSimpleExpressionSymbol.of(value.getDividend().intValue()));
        }
        else {
            parameter.setExpression(ArchSimpleExpressionSymbol.of(value.doubleValue()));
        }
    }

    public void setParameter(String name, boolean value){
        ParameterSymbol parameter = getParameterOrError(name);
        parameter.setExpression(ArchSimpleExpressionSymbol.of(value));
    }

    public void setParameter(String name, int value){
        ParameterSymbol parameter = getParameterOrError(name);
        parameter.setExpression(ArchSimpleExpressionSymbol.of(value));
    }

    public void setParameter(String name, double value){
        ParameterSymbol parameter = getParameterOrError(name);
        parameter.setExpression(ArchSimpleExpressionSymbol.of(value));
    }

    public void setParameter(String name, String value){
        ParameterSymbol parameter = getParameterOrError(name);
        parameter.setExpression(ArchSimpleExpressionSymbol.of(value));
    }


    public CNNArchCompilationUnitSymbol preResolveDeepCopy(){
        return preResolveDeepCopy(getName());
    }

    public CNNArchCompilationUnitSymbol preResolveDeepCopy(String newName){
        CNNArchCompilationUnitSymbol copy = new CNNArchCompilationUnitSymbol(newName);
        if (getAstNode().isPresent()){
            copy.setAstNode(getAstNode().get());
        }

        List<ParameterSymbol> parameterCopies = new ArrayList<>(getParameters().size());
        for (ParameterSymbol param : getParameters()){
            ParameterSymbol paramCopy = param.deepCopy();
            parameterCopies.add(paramCopy);
            paramCopy.putInScope(copy.getSpannedScope());
        }
        copy.setParameters(parameterCopies);

        List<IODeclarationSymbol> ioCopies = new ArrayList<>(getIoDeclarations().size());
        for (IODeclarationSymbol ioDeclaration : getIoDeclarations()){
            IODeclarationSymbol ioCopy = (IODeclarationSymbol) ioDeclaration.preResolveDeepCopy();
            ioCopies.add(ioCopy);
            ioCopy.putInScope(copy.getSpannedScope());
        }
        copy.setIoDeclarations(ioCopies);

        ArchitectureSymbol architecture = getArchitecture().preResolveDeepCopy(copy.getSpannedScope());
        copy.setArchitecture(architecture);
        Utils.recursiveSetResolvingFilters(copy.getSpannedScope(), getSpannedScope().getResolvingFilters());
        return copy;
    }
}
