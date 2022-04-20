package de.thesis.consumer.backend.persistence.repository;

import de.thesis.consumer.backend.persistence.entity.DataRowEntity;
import org.springframework.data.repository.CrudRepository;
import org.springframework.stereotype.Repository;

import java.util.List;

@Repository
public interface SpringDataDataRowCrudRepository extends CrudRepository<DataRowEntity, Long> {
	List<DataRowEntity> findAll();
}
