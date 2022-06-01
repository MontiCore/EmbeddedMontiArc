package ports;

public interface DatasetDeletionExecutionPort extends DsExecutionPort {

	boolean deleteDataset(String datasetId);
}
