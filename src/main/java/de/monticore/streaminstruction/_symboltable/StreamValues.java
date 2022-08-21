/* (c) https://github.com/MontiCore/monticore */
package de.monticore.streaminstruction._symboltable;

import de.monticore.streaminstruction._symboltable.IStreamValue;

import java.util.ArrayList;
import java.util.List;

/**
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
