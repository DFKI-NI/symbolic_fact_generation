# symbolic_fact_generation

[![Actively Maintained](https://img.shields.io/badge/Maintenance%20Level-Actively%20Maintained-green.svg)](https://gist.github.com/cheerfulstoic/d107229326a01ff0f333a1d3476e068d)

Transforms robot sensor data into symbolic facts for planning. Generates facts for [mobipick_labs](https://github.com/DFKI-NI/mobipick_labs/). Facts about objects are generated
using the poses of the tables (given by a config file) and the results of the object pose estimation created by DOPE (Deep Object Pose Estimation).

# Requirements

The symbolic fact generation assumes the poses of the detected objects to be stored in the [pose_selector](https://github.com/DFKI-NI/pose_selector/).

# Usage
## Launch Files

Use the provided launch files to launch the fact generation:

    roslaunch symbolic_fact_generation fact_generation.launch

Launches the mobipick_labs fact generation and publishes the facts on the following topics:

    /fact_publisher/facts symbolic_fact_generation/Facts
    /fact_publisher/changed_facts symbolic_fact_generation/ChangedFacts

The first topic publishes all current facts every second, while the second topic only publishes changes in facts, new facts and removed facts.

## Python Import

Another way of generating the facts specified in the config file is to import the fact generating class in python:

    from symbolic_fact_generation.fact_generation_with_config import FactGenerationWithConfig

    fact_generator = FactGenerationWithConfig(config_path)

    current_facts = fact_generator.generate_facts()
    specific_facts = fact_generator.generate_facts_with_name(fact_name)

```generate_facts()``` returns all facts specified in the config file, while ```generate_facts_with_name(fact_name)``` only returns the facts with the given name,
which also needs to be specified in the config file.

You can also import the fact generating class directly. For example only using the "on" facts:

    from symbolic_fact_generation.on_fact_generator import OnGenerator
    
    on_generator = OnGenerator(fact_name, objects_of_interest, container_objects, pose_selector_query_srv_str, planning_scene_param)
    current_facts = on_generator.generate_facts()

Every Generator class has a ```generate_facts()``` function, which returns a list of ```Fact``` objects. `OnGenerator` can optionally use Pose Selector's get-all service so newly inserted open-set classes participate without being known in its static class list.

`GripperHasObjectGenerator` monitors the configured gripper joint and emits the
zero-argument `gripper_has_object()` fact when the gripper stops short of its
fully closed position, excluding the fully open position. It is intended to be
evaluated after a close command. An optional absolute effort threshold can
confirm contact in addition to the position check. The MobiPick configuration
requires `0.1 Nm`, which prevents a gripper resting at a partially open command
from being mistaken for a grasp. Leave the threshold at zero when joint effort
is unavailable.

For MobiPick, the open tolerance includes both the real gripper's `0.0 rad`
reading and the approximately `0.049 rad` joint position produced when the
simulated calibrated controller is commanded to its `0.140 m` open gap.

## Creating Custom Fact Generation

To create different facts, a fact generator class with a ```generate_facts()``` function is needed:
    
    generate_facts(self) -> List[Facts]

In the classes ```__init__``` constructor needed sensor information or other connections can be established for the fact generation to work.

Additionally in a config file the name of the fact, the python module as well as which python class generates the fact needs to be provided.
Parameters for the classes ```__init__``` constructor can also be provided (e.g. topic names, object names, etc.) in this file.
An example can be seen here: [config/facts_config.yaml](https://github.com/DFKI-NI/symbolic_fact_generation/blob/main/config/facts_config.yaml).

# Contact

Marian Renz - marian.renz@dfki.de

Marc Vinci - marc.vinci@dfki.de
