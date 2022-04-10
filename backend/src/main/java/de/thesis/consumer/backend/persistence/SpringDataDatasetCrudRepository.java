package de.thesis.consumer.backend.persistence;

import de.thesis.consumer.backend.entities.Dataset;
import org.springframework.data.repository.CrudRepository;
import org.springframework.stereotype.Repository;

import java.util.List;
import java.util.UUID;

@Repository
public interface SpringDataDatasetCrudRepository extends CrudRepository<Dataset, UUID> {
	List<Dataset> findAll();
}
