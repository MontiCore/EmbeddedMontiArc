package ports;

public interface LocalLoggingExecutionPoint extends PolicyExecutionPoint{

	boolean log(String datasetId);
}
