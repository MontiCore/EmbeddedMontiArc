package ports;

public interface RemoteLoggingExecutionPoint extends PolicyExecutionPoint {

	boolean log(String url, String datasetId);
}
