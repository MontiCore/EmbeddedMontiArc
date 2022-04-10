package de.thesis.consumer.backend.controllers;

import de.thesis.consumer.backend.domain.DatasetService;
import de.thesis.consumer.backend.entities.Dataset;
import lombok.AllArgsConstructor;
import org.springframework.web.bind.annotation.CrossOrigin;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.util.List;

@RestController
@RequestMapping("/datasets")
@AllArgsConstructor
@CrossOrigin(origins = "*")
public class DatasetController {

	private DatasetService service;

	@GetMapping
	public List<Dataset> getAllOffers() {
		return service.getAllDatasets();
	}
}
