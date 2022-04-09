package de.thesis.consumer.backend.persistence;

import de.thesis.consumer.backend.entities.Dataset;
import org.springframework.data.repository.CrudRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface SpringDataDatasetCrudRepository extends CrudRepository<Dataset, Long> {
}
