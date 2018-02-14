/**
 * ******************************************************************************
 * MontiCAR Modeling Family, www.se-rwth.de
 * Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 * All rights reserved.
 * <p>
 * This project is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 * <p>
 * You should have received a copy of the GNU Lesser General Public
 * License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.streamunits._symboltable;

import java.util.Optional;

/**
 * @author Sascha Schneiders
 */
public class StreamInstruction {
    protected Optional<StreamValuePrecision> streamValue = Optional.empty();
    protected Optional<StreamCompare> streamCompare = Optional.empty();
    protected Optional<StreamValueAtTick> streamValueAtTick = Optional.empty();
    protected Optional<StreamValues> streamValues = Optional.empty();

    public StreamInstruction() {

    }

    public StreamInstruction(StreamValuePrecision streamValue) {
        this.streamValue = Optional.of(streamValue);
    }

    public StreamInstruction(StreamValueAtTick valueAtTick) {
        this.streamValueAtTick = Optional.of(valueAtTick);
    }

    public StreamInstruction(StreamValues valuesAtTick) {
        this.streamValues = Optional.of(valuesAtTick);
    }

    public StreamInstruction(StreamCompare streamCompare) {
        this.streamCompare = Optional.of(streamCompare);
    }

    public Optional<StreamValuePrecision> getStreamValue() {
        return streamValue;
    }

    public void setStreamValue(StreamValuePrecision streamValue) {
        this.streamValue = Optional.of(streamValue);
    }

    public Optional<StreamCompare> getStreamCompare() {
        return streamCompare;
    }

    public void setStreamCompare(StreamCompare streamCompare) {
        this.streamCompare = Optional.of(streamCompare);
    }

    public void setStreamValue(Optional<StreamValuePrecision> streamValue) {
        this.streamValue = streamValue;
    }

    public void setStreamCompare(Optional<StreamCompare> streamCompare) {
        this.streamCompare = streamCompare;
    }

    public Optional<StreamValueAtTick> getStreamValueAtTick() {
        return streamValueAtTick;
    }

    public void setStreamValueAtTick(Optional<StreamValueAtTick> streamValueAtTick) {
        this.streamValueAtTick = streamValueAtTick;
    }

    public Optional<StreamValues> getStreamValues() {
        return streamValues;
    }

    public void setStreamValues(Optional<StreamValues> streamValues) {
        this.streamValues = streamValues;
    }
}
