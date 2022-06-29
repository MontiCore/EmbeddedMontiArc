package de.thesis.provider.backend.commands;


import de.thesis.provider.backend.entities.Metadata;
import de.thesis.provider.backend.csv.DataRow;
import lombok.Value;

import java.util.List;

@Value
public class CreateOfferCommand {
	Metadata metadata;
	List<DataRow> data;
}
