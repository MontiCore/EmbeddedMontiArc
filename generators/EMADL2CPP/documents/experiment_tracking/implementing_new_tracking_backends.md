# Implementing new Tracking Backends
The first step of introducing a new tracking backend is the creation of a class implementing the [RunTracker interface](src/main/java/de/monticore/mlpipelines/tracking/tracker/RunTracker.java). To maintain a coherent nomenclature, its name should start with the name of the tracking framework, and end with "Tracker". The class must overwrite all methods defined in the interface, processing the provided metadata as desired. For that purpose, it may utilize a Java API offered by many tracking frameworks. If the tracking backend needs to be configurable, its constructor must receive an `ASTConfigurationEntry`, which can then be used to extract the required information. This process can be simplified by auxiliary functions provided through the [ASTConfLangHelper](src/main/java/de/monticore/mlpipelines/tracking/helper/ASTConfLangHelper.java). To synchronize the configuration of this class with its Python counterpart, whose implementation will be covered at the end of this guide, the method `getPythonParams()` must be overwritten to return a map, linking attribute identifiers to their respective values. These attributes may contain information like run IDs or the URI of a remote tracking server. Since the attribute identifiers are used as CLI options for the Python script, they must not contain spaces and need to be unique among all implemented tracking backends. Avoiding collisions can easily be achieved by prepending the name of the tracking backend to the attribute identifier. If no attributes have to be synchronized, an empty map may be returned. The [StdOutTracker](src/main/java/de/monticore/mlpipelines/tracking/tracker/StdOutTracker.java)(simple) and [MLflowTracker](src/main/java/de/monticore/mlpipelines/tracking/tracker/MLflowTracker.java)(complex) are examples of valid implementations of the RunTracker interface.

For our tracking system to recognize and automatically instantiate the newly created tracking backend, two files have to be modified. First, a new constant has to be appended to the enum [TrackingBackend](src/main/java/de/monticore/mlpipelines/tracking/TrackingBackend.java), defining how a configuration entry must be named to enable the tracking backend. While his constant can have any value, naming it after the implemented tracking framework, is considered best practice for maintaining a consistent naming scheme. Additionally, two lines of code have to be inserted into the `createTracker()` method offered by the [TrackerFactory](src/main/java/de/monticore/mlpipelines/tracking/TrackerFactory.java). Among other things, this function iterates over all enabled backends, creating corresponding instances of RunTracker. Therefore, a new case has to be introduced to the switch statement, handling the instantiation of the new tracking backend. The TrackingBackend enum and `createTracker()` method already contain the required entries/cases for the StdOutTracker and MLflowTracker respectively.

To complete the process of integrating a new tracking backend into the generator, a Python counterpart to the RunTracker previously implemented in Java code has to be created. This is achieved by appending a new class to the [RunTracker.py](src/main/resources/experiments/tracking/RunTracker.py) file. While this class can have any name, we recommend selecting the same name as for the Java class. It must inherit from the abstract `RunTracker` class (present in the same file), and therefore implement the `log_metric()` function for processing the provided performance metrics. The constructor of this class receives a namespace containing all CLI arguments, individually accessible via the attribute identifiers defined in the `getPythonParameters()` method. Finally, akin to the modification in `createTracker()` (see above), the automatic instantiation of new tracking backends needs to be enabled by inserting two lines of code into the `extract_run_trackers()` function provided by the MultiBackendTracker class (also contained in RunTracker.py). This file already contains the classes `StdOutTracker` and `MLflowTracker`, serving as the Python counterpart to the identically named Java classes.