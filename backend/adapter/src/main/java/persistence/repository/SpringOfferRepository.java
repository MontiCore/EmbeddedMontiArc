package persistence.repository;

import org.springframework.data.jpa.repository.Modifying;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.CrudRepository;
import org.springframework.data.repository.query.Param;
import org.springframework.stereotype.Repository;
import persistence.entity.OfferEntity;

import javax.transaction.Transactional;
import java.util.UUID;

@Repository
public interface SpringOfferRepository extends CrudRepository<OfferEntity, UUID> {
	@Transactional
	@Modifying
	@Query(value =
			"UPDATE data_row SET offer = null WHERE offer = :offerId ; " +
					"DELETE FROM offer WHERE id = :offerId",
			nativeQuery = true)
	void deleteByIdWithoutCascading(@Param("offerId") UUID offerId);

	@Transactional
	void deleteById(UUID offerId);
}
