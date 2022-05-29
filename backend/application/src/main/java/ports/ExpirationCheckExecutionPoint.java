package ports;

public interface ExpirationCheckExecutionPoint extends PolicyExecutionPoint {

	boolean removeExpiredDatasets();
}
