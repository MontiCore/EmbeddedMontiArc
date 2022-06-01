package ports;

public interface ExpirationCheckExecutionPort extends DsExecutionPort {

	boolean removeExpiredDatasets();
}
