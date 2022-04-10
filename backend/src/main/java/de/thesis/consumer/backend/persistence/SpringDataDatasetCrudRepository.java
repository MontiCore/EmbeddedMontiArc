package de.thesis.consumer.backend.persistence;

import de.thesis.consumer.backend.entities.Dataset;
import org.springframework.data.repository.CrudRepository;
import org.springframework.stereotype.Repository;

import java.util.List;

@Repository
public interface SpringDataDatasetCrudRepository extends CrudRepository<Dataset, Long> {
	List<Dataset> findAll();
}
