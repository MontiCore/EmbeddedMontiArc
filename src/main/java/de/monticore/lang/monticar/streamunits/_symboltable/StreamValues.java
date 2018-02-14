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

import java.util.ArrayList;
import java.util.List;

/**
 * @author Sascha Schneiders
 *         This class stores the rows of a stream value matrix/array
 */
public class StreamValues {

    protected List<List<IStreamValue>> streamValues = new ArrayList<>();

    public StreamValues() {

    }

    public StreamValues(List<IStreamValue> streamValues) {
        this.streamValues.add(streamValues);
    }

    public List<IStreamValue> getStreamValues(int rowIndex) {
        return streamValues.get(rowIndex);
    }

    /**
     *
     * @param rowIndex starts at 0
     * @param columnIndex starts at 0
     * @return the element in row rowIndex and column columnIndex
     */
    public IStreamValue getStreamValue(int rowIndex, int columnIndex) {
        return streamValues.get(rowIndex).get(columnIndex);
    }

    public void add(List<IStreamValue> streamValues) {
        this.streamValues.add(streamValues);
    }

    public void setStreamValues(List<List<IStreamValue>> streamValues) {
        this.streamValues = streamValues;
    }

    /**
     * @return amount of rows
     */
    public int getRowDimension() {
        return streamValues.size();
    }

    /**
     * @return amount of columns
     */
    public int getColumnDimension() {
        if (streamValues.size() == 0) {
            return 0;
        }
        return streamValues.get(0).size();
    }

}
