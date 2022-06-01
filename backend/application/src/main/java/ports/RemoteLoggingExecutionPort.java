package ports;

public interface RemoteLoggingExecutionPort extends DsExecutionPort {

	boolean log(String url, String datasetId);
}
