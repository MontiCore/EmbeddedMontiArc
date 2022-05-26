package entity;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Value;

import java.time.LocalDate;
import java.util.List;
import java.util.UUID;

@Data
@AllArgsConstructor
public class Offer {
	UUID id;
	Metadata metadata;
	List<DataRow> data;
}
