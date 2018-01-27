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
package de.monticore.lang.monticar.emadl.helper;

/*
  contains toString functions which translate AST nodes into a String for generation
 */
public interface ASTPrinter {

    /*static String toString(ASTNumber number) {
        if (number.getUnitNumber().isPresent()) {
            return number.getUnitNumber().get().toString();
        } else if (number.getFloatPointUnitNumber().isPresent()) {
            return number.getFloatPointUnitNumber().get().getTFloatPointUnitNumber();
        } else if (number.getComplexNumber().isPresent()) {
            return number.getComplexNumber().get().toString();
        } else if (number.getHexUnitNumber().isPresent()) {
            return number.getHexUnitNumber().get().getTHexUnitNumber();
        } else {
            return null;
        }
    }*/

    /*static String toString(ASTTuple tuple) {
        List<String> stringList = new LinkedList<>();

        for (ASTNumber number : tuple.getValues()) {
            stringList.add(toString(number));
        }

        String res = Joiners.COMMA.join(stringList);
        res = "(" + res + ")";
        return res;
    }*/

    /*static String toString(ASTArgumentRhs rhs) {
        if (rhs == null){
            return null;
        }

        if (rhs.getType().isPresent()) {
            return rhs.getType().get().getName().get();
        }
        else if(rhs.getNumber().isPresent()) {
            return toString(rhs.getNumber().get());
        }
        else if(rhs.getTuple().isPresent()) {
            return toString(rhs.getTuple().get());
        }
        else if (rhs.getBooleanVal().isPresent()){
            return rhs.getBooleanVal().get().name().toLowerCase();
        }
        else {
            //should never be reached
            return null;
        }
    }

    static String toString(ASTParameterRhs rhs) {

        if (rhs == null){
            return null;
        }

        if (rhs.getStringVal().isPresent()) {
            return rhs.getStringVal().get();
        }
        else if(rhs.getNumber().isPresent()) {
            return toString(rhs.getNumber().get());
        }
        else if (rhs.getBooleanVal().isPresent()){
            return rhs.getBooleanVal().get();
        }
        else {
            return rhs.getRef().get();
        }
    }*/

}
