package persistence.entity;


import lombok.Getter;
import lombok.Setter;

import javax.persistence.*;

@Entity
@Table(name = "policy")
@Getter
@Setter
public class PolicyEntity {
	@Id
	private String id;

	@Column(columnDefinition = "TEXT")
	private String rawValue;
}

