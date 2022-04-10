package de.thesis.consumer.backend.persistence;

import de.thesis.consumer.backend.domain.OfferRepository;
import de.thesis.consumer.backend.entities.Offer;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Repository;

import java.util.List;
import java.util.UUID;

@Repository
@AllArgsConstructor
public class OfferRepositoryImpl implements OfferRepository {

	private SpringDataOfferCrudRepository repository;

	@Override
	public void save(Offer offer) {
		repository.save(offer);
	}

	@Override
	public List<Offer> findAll() {
		return repository.findAll();
	}

	@Override
	public Offer findBy(UUID offerId) {
		return repository.findById(offerId).orElse(null);
	}
}
