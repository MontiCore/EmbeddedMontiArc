package persistence.repository;

import org.springframework.data.jpa.repository.Modifying;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.CrudRepository;
import org.springframework.data.repository.query.Param;
import org.springframework.stereotype.Repository;
import persistence.entity.DatasetEntity;

import javax.transaction.Transactional;
import java.time.LocalDateTime;
import java.util.UUID;

@Repository
public interface SpringDatasetRepository extends CrudRepository<DatasetEntity, UUID> {

	@Transactional
	@Modifying
	@Query(value =
			"INSERT INTO dataset (id, bought_at, metadata_id, offer_id) VALUES (:datasetId, :boughtAt, :metadataId, :offerId);" +
					"UPDATE data_row SET dataset = :datasetId WHERE offer = :offerId",
			nativeQuery = true)
	void save(
			@Param("datasetId") UUID datasetId,
			@Param("boughtAt") LocalDateTime boughtAt,
			@Param("metadataId") int metadataId,
			@Param("offerId") UUID offerId
	);

	@Transactional
	void deleteByOfferId(UUID offerId);
}

