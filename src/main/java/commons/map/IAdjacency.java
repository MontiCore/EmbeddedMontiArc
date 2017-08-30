package commons.map;

/**
 * Created by lukas on 31.01.17.
 */
public interface IAdjacency {
    public abstract IControllerNode getNode1();

    public abstract IControllerNode getNode2();

    public abstract double getDistance();
}
