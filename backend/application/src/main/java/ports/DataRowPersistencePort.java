package ports;

import entity.DataRow;

public interface DataRowPersistencePort {
	void saveAll(Iterable<DataRow> data);
}
