/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.streamunits._symboltable;

import de.monticore.lang.monticar.streamunits._ast.ASTValueAtTick;
import org.jscience.mathematics.number.Rational;

import java.util.Optional;

import static de.monticore.numberunit.Rationals.doubleToRational;

/**
 */
public class StreamValueAtTick implements IStreamValue {
    protected String name;
    protected Optional<Rational> singleValue = Optional.empty();
    protected Optional<Rational> lowerBound = Optional.empty();
    protected Optional<Rational> upperBound = Optional.empty();

    public StreamValueAtTick() {

    }

    public StreamValueAtTick(ASTValueAtTick valueAtTick) {
        this.name = valueAtTick.getName();
        if (valueAtTick.getValueOpt().isPresent())
            this.singleValue = Optional.of(doubleToRational(valueAtTick.getValueOpt().get().getNumber().get()));
        if (valueAtTick.getLowerBoundOpt().isPresent())
            this.lowerBound = Optional.of(doubleToRational(valueAtTick.getLowerBoundOpt().get().getNumber().get()));
        if (valueAtTick.getUpperBoundOpt().isPresent())
            this.upperBound = Optional.of(doubleToRational(valueAtTick.getUpperBoundOpt().get().getNumber().get()));

    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public Optional<Rational> getSingleValue() {
        return singleValue;
    }

    public void setSingleValue(Optional<Rational> singleValue) {
        this.singleValue = singleValue;
    }

    public Optional<Rational> getLowerBound() {
        return lowerBound;
    }

    public void setLowerBound(Optional<Rational> lowerBound) {
        this.lowerBound = lowerBound;
    }

    public Optional<Rational> getUpperBound() {
        return upperBound;
    }

    public void setUpperBound(Optional<Rational> upperBound) {
        this.upperBound = upperBound;
    }

    @Override
    public boolean isStreamValueAtTick() {
        return true;
    }

    @Override
    public void visit(NamedStreamUnitsSymbol streamUnitsSymbol) {
        streamUnitsSymbol.add(this);
    }
}
