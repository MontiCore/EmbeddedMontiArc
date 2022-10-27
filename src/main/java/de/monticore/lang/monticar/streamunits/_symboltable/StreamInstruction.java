/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.streamunits._symboltable;

import java.util.Optional;

/**
 */
public class StreamInstruction {
    protected Optional<StreamValuePrecision> streamValue = Optional.empty();
    protected Optional<StreamCompare> streamCompare = Optional.empty();
    protected Optional<StreamValueAtTick> streamValueAtTick = Optional.empty();
    protected Optional<StreamValues> streamValues = Optional.empty();
    protected Optional<ImagePath> imagePath = Optional.empty();

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

    public StreamInstruction(ImagePath imagePath) { this.imagePath = Optional.of(imagePath); }

    public Optional<StreamValuePrecision> getStreamValue() {
        return streamValue;
    }

    public void setStreamValue(Optional<StreamValuePrecision> streamValue) {
        this.streamValue = streamValue;
    }

    public void setStreamValue(StreamValuePrecision streamValue) {
        this.streamValue = Optional.of(streamValue);
    }

    public Optional<StreamCompare> getStreamCompare() {
        return streamCompare;
    }

    public void setStreamCompare(Optional<StreamCompare> streamCompare) {
        this.streamCompare = streamCompare;
    }

    public void setStreamCompare(StreamCompare streamCompare) {
        this.streamCompare = Optional.of(streamCompare);
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

    public Optional<ImagePath> getImagePath() { return imagePath; }

    public void setImagePath(Optional<ImagePath> imagePath) { this.imagePath = imagePath; }
}
