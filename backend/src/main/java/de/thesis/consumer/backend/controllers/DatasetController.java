package de.thesis.consumer.backend.controllers;

import de.thesis.consumer.backend.domain.OfferService;
import de.thesis.consumer.backend.domain.InvalidPolicyException;
import de.thesis.consumer.backend.entities.Offer;
import lombok.AllArgsConstructor;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.server.ResponseStatusException;

import java.util.List;

@RestController
@RequestMapping("/datasets")
@AllArgsConstructor
@CrossOrigin(origins = "*")
public class DatasetController {

	private OfferService service;

	@PostMapping(consumes = MediaType.APPLICATION_JSON_VALUE)
	@ResponseStatus(HttpStatus.OK)
	public void offer(@RequestBody Offer offer) {
		try {
			service.offerDataset(offer);
		} catch (InvalidPolicyException e) {
			throw new ResponseStatusException(HttpStatus.BAD_REQUEST, "Dataset policy is invalid");
		}
	}

	@GetMapping
	public List<Offer> getAllOffers() {
		return service.getAllOffers();
	}
}
