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
package de.monticore.lang.numberunit;

import org.jscience.mathematics.number.Rational;

import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.Locale;

/**
 * Created by michaelvonwenckstern on 11.02.17.
 */
public class Rationals {
    public static Rational create(String s) {
        s = s.replace(" ", "");
        if (s.contains(".")) {
            return doubleToRational(Double.valueOf(s));
        }
        if (s.contains("/"))
            return Rational.valueOf(s);
        return Rational.valueOf(Long.valueOf(s), 1);
    }

    public static Rational doubleToRational(double value) {
        //String tmp = Double.toString(value);
        DecimalFormat df = new DecimalFormat("0", DecimalFormatSymbols.getInstance(Locale.ENGLISH));
        df.setMaximumFractionDigits(340); //340 = DecimalFormat.DOUBLE_FRACTION_DIGITS
        String tmp = df.format(value);
        String[] rational = tmp.split("\\.");
        long denom = 1;
        long num = Long.valueOf(rational[0] + "");
        if (rational.length > 1) {
            denom = (long) Math.pow(10, rational[1].length());
            //System.out.println(denom);
            num = Long.valueOf(rational[0] + "" + rational[1]);
            //System.out.println(num);
        }
        return Rational.valueOf(num, denom);
    }
}
