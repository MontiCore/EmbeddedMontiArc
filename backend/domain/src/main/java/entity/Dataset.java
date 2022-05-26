package entity;

import lombok.Data;

import java.time.LocalDate;
import java.time.LocalDateTime;
import java.util.List;
import java.util.UUID;

@Data
public class Dataset {
	private UUID id;
	private Metadata metadata;
	private List<DataRow> data;
	private LocalDateTime boughtAt;
}
