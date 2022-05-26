package ports;

import entity.DataRow;
import entity.Dataset;

import javax.xml.crypto.Data;
import java.util.UUID;

public interface DataPersistencePort {

	Iterable<Dataset> findById(UUID id);

	Iterable<Dataset> findAll();

	void save(Dataset dataset);


}
