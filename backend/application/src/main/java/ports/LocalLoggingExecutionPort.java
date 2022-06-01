package ports;

public interface LocalLoggingExecutionPort extends DsExecutionPort {

	boolean log(String datasetId);
}
