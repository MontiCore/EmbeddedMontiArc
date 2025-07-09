package de.thesis.provider.backend.dto;

import de.thesis.provider.backend.csv.DataRow;
import lombok.Getter;
import lombok.Setter;

import java.util.List;
import java.util.UUID;

@Getter
@Setter
public class Offer {
	private UUID id;
	private Metadata metadata;
	private List<DataRow> data;
}
