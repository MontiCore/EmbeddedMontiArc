/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.solutionValidation;

import java.util.Collection;
import java.util.Optional;

public class PortRangeValidationViewModel {
    private final Collection<PortLowerRangeViewModel> portLowerRangeViewModels;
    private final Collection<PortUpperRangeViewModel> portUpperRangeViewModels;

    private final Optional<PortLowerRangeViewModel> portInvalidLowerRangeViewModel;
    private final Optional<PortUpperRangeViewModel> portInvalidUpperRangeViewModel;

    public PortRangeValidationViewModel(Collection<PortLowerRangeViewModel> portLowerRangeViewModels,
                                        Collection<PortUpperRangeViewModel> portUpperRangeViewModels) {
        this.portUpperRangeViewModels = portUpperRangeViewModels;
        this.portLowerRangeViewModels = portLowerRangeViewModels;
        portInvalidLowerRangeViewModel = Optional.empty();
        portInvalidUpperRangeViewModel = Optional.empty();
    }

    public PortRangeValidationViewModel(Collection<PortLowerRangeViewModel> portLowerRangeViewModels,
                                        Collection<PortUpperRangeViewModel> portUpperRangeViewModels,
                                        PortLowerRangeViewModel portInvalidLowerRangeViewModel) {
        this.portUpperRangeViewModels = portUpperRangeViewModels;
        this.portLowerRangeViewModels = portLowerRangeViewModels;
        this.portInvalidLowerRangeViewModel = Optional.of(portInvalidLowerRangeViewModel);
        this.portInvalidUpperRangeViewModel = Optional.empty();
    }

    public PortRangeValidationViewModel(Collection<PortLowerRangeViewModel> portLowerRangeViewModels,
                                        Collection<PortUpperRangeViewModel> portUpperRangeViewModels,
                                        PortUpperRangeViewModel portInvalidUpperRangeViewModel) {
        this.portUpperRangeViewModels = portUpperRangeViewModels;
        this.portLowerRangeViewModels = portLowerRangeViewModels;
        this.portInvalidLowerRangeViewModel = Optional.empty();
        this.portInvalidUpperRangeViewModel = Optional.of(portInvalidUpperRangeViewModel);
    }

    public Collection<PortLowerRangeViewModel> getPortLowerRangeViewModels() {
        return portLowerRangeViewModels;
    }

    public Collection<PortUpperRangeViewModel> getPortUpperRangeViewModels() {
        return portUpperRangeViewModels;
    }

    public boolean isPresentPortInvalidLowerRangeViewModel() {
        return portInvalidLowerRangeViewModel.isPresent();
    }

    public PortLowerRangeViewModel getPortInvalidLowerRangeViewModel() {
        return portInvalidLowerRangeViewModel.orElse(null);
    }

    public boolean isPresentPortInvalidUpperRangeViewModel() {
        return portInvalidUpperRangeViewModel.isPresent();
    }

    public PortUpperRangeViewModel getPortInvalidUpperRangeViewModel() {
        return portInvalidUpperRangeViewModel.orElse(null);
    }
}
