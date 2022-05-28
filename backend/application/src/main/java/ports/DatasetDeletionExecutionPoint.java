package ports;

public interface DatasetDeletionExecutionPoint extends PolicyExecutionPoint{

	boolean deleteDataset(String datasetId);
}
