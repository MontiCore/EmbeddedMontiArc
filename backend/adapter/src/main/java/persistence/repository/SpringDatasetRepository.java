package persistence.repository;

import org.springframework.data.repository.CrudRepository;
import org.springframework.stereotype.Repository;
import persistence.entity.DatasetEntity;

import java.util.UUID;

@Repository
public interface SpringDatasetRepository extends CrudRepository<DatasetEntity, UUID> {
}
