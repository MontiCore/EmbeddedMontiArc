package de.thesis.provider.backend.controller;

import de.thesis.provider.backend.csv.CsvReader;
import de.thesis.provider.backend.csv.TruckData;
import org.springframework.web.bind.annotation.*;

import java.io.IOException;
import java.util.List;

@RestController
@RequestMapping("truckdata")
@CrossOrigin(origins = "*")
public class TruckDataController {

	@GetMapping
	public List<TruckData> getTruckData() throws IOException {
		CsvReader reader = new CsvReader();
		return reader.getCsvData();
	}
}
