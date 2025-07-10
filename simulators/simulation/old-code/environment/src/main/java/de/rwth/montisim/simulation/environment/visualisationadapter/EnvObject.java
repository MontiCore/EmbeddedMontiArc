/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.visualisationadapter;

import de.rwth.montisim.simulation.environment.visualisationadapter.interfaces.EnvTag;

import java.util.List;

/**
 * Created by lukas on 08.01.17.
 *
 * A superclass for all EnvironmentObjects
 */
public class EnvObject {

    protected List<EnvNode> nodes;
    protected EnvTag tag;

    protected long osmId;


    public EnvObject(List<EnvNode> nodes, EnvTag tag) {
        this.nodes = nodes;
        this.tag = tag;
    }

    public EnvObject(List<EnvNode> nodes, EnvTag tag, long osmId) {
        this(nodes, tag);
        this.osmId = osmId;
    }

    public List<EnvNode> getNodes() {
        return this.nodes;
    }

    public EnvTag getTag() {
        return this.tag;
    }

    public long getOsmId() {
        return this.osmId;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        EnvObject that = (EnvObject) o;

        if (getOsmId() != that.getOsmId()) return false;
        if (!getNodes().equals(that.getNodes())) return false;
        return getTag() == that.getTag();

    }

    @Override
    public int hashCode() {
        int result = getNodes().hashCode();
        result = 31 * result + getTag().hashCode();
        result = 31 * result + (int) (getOsmId() ^ (getOsmId() >>> 32));
        return result;
    }
}
