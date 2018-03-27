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
package de.monticore.lang.numberunit._ast;

import de.monticore.lang.numberunit.Rationals;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;

/**
 * Created by michaelvonwenckstern on 11.02.17.
 */
public class ASTComplexNumber extends ASTComplexNumberTOP {
    protected Rational real = Rational.ZERO;
    protected Rational img = Rational.ZERO;


    public Rational getReal() {
        return real;
    }

    public Rational getImg() {
        return img;
    }

    public ASTComplexNumber() {

        super();
    }

    public ASTComplexNumber(String tComplexNumber) {
        super(tComplexNumber);
    }

    @Override
    public void setTComplexNumber(String tComplexNumber) {
        super.setTComplexNumber(tComplexNumber);

        int pos = tComplexNumber.indexOf("+");
        if (pos == -1) {
            pos = tComplexNumber.indexOf("-", 1); // -3 - 7i (do not want the first -)
        }

        real = Rationals.create(tComplexNumber.substring(0, pos));
        String sImg = tComplexNumber.substring(pos, tComplexNumber.length() - 1);
        img = Rationals.create(sImg);
    }

    public void setReal(Rational real) {
        this.real = real;
    }

    public void setImg(Rational img) {
        this.img = img;
    }
    
    @Override
    public String toString() {
        return getTComplexNumber();
    }

    public  ASTComplexNumber deepClone()   {

        return deepClone(_construct());

    }



    public  ASTComplexNumber deepClone(ASTComplexNumber result)   {
        Log.errorIfNull(result, "0xA7006_285 Parameter 'result' must not be null.");

        super.deepClone(result);

        result.setReal(this.getReal());
        result.setImg(this.getImg());

        return result;

    }
}
