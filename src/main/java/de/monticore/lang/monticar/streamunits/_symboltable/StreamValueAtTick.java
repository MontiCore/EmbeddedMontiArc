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
package de.monticore.lang.monticar.streamunits._symboltable;

import de.monticore.lang.monticar.streamunits._ast.ASTValueAtTick;
import org.jscience.mathematics.number.Rational;

import java.util.Optional;

/**
 * @author Sascha Schneiders
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
            this.singleValue = Optional.of(valueAtTick.getValueOpt().get().getNumber().get());
        if (valueAtTick.getLowerBoundOpt().isPresent())
            this.lowerBound = Optional.of(valueAtTick.getLowerBoundOpt().get().getNumber().get());
        if (valueAtTick.getUpperBoundOpt().isPresent())
            this.upperBound = Optional.of(valueAtTick.getUpperBoundOpt().get().getNumber().get());

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
