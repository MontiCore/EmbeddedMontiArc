package de.thesis.consumer.backend.controllers;

import de.thesis.consumer.backend.domain.DatasetService;
import de.thesis.consumer.backend.domain.InvalidPolicyException;
import de.thesis.consumer.backend.entities.Dataset;
import lombok.AllArgsConstructor;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.server.ResponseStatusException;

@RestController
@RequestMapping("/datasets")
@AllArgsConstructor
public class DatasetController {

	private DatasetService service;

	@PostMapping(consumes = MediaType.APPLICATION_JSON_VALUE)
	@ResponseStatus(HttpStatus.OK)
	public void offerDataset(@RequestBody Dataset dataset) {
		try {
			this.service.offerDataset(dataset);
		} catch (InvalidPolicyException e) {
			throw new ResponseStatusException(HttpStatus.BAD_REQUEST, "Dataset policy is invalid");
		}
	}
}
