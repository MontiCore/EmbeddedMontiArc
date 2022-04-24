package de.thesis.consumer.backend.domain.repository;

import de.thesis.consumer.backend.domain.exception.PolicyNotFoundException;
import de.thesis.consumer.backend.domain.model.Offer;

import java.util.List;
import java.util.UUID;

public interface OfferRepository {

	void save(Offer offer) throws PolicyNotFoundException;

	List<Offer> findAll();

	Offer findBy(UUID offerId);
}
