package de.thesis.consumer.backend.persistence.repository;

import de.thesis.consumer.backend.persistence.entity.DatasetEntity;
import org.springframework.data.repository.CrudRepository;
import org.springframework.stereotype.Repository;

import java.util.List;
import java.util.UUID;

@Repository
public interface SpringDataDatasetCrudRepository extends CrudRepository<DatasetEntity, UUID> {
	List<DatasetEntity> findAll();
}
