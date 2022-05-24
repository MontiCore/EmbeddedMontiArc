package persistence.entity;

import lombok.Getter;
import lombok.Setter;

import javax.persistence.Entity;
import javax.persistence.Id;
import javax.persistence.Table;
import java.util.UUID;

@Entity
@Table(name = "offer")
@Getter
@Setter
public class OfferEntity {

	@Id
	private UUID id;
	private String title;
}
