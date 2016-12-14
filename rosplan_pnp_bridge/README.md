# The ROSplan PNP bridge

This module bridges the functionality of PNP to allow the usage with ROSPlan as a planner. It interprets the generated plan and calls the pnp generation service to create a PNP and send it off for execution. It also receives feedback from the PNP server and triggers replanning if necessary.

Return values of PNP actions are interpreted based on the following scheme:

```
predicate_name__value_1__value_2
```

which will automatically be added or removed from the KB (depending on its truth value) in the correct format:

```
(predicate value_1 value_2)
```

Only primitive types for values are allowd, e.g. string, int, float, etc. Predicates have to start with a letter and are always lower case. Values are separated using two underscores `__`.

## How to write a ROSPlan PNP action

Actions are defined in the PDDL file used as the planning domain. Let's assume a simple example:

```pddl
(define (domain pepper_demo)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
        id
        interactant_id
        ...
)

(:predicates
        (found_interactant ?i - interactant_id ?h - id)
        (free_interactant_id ?i - interactant_id)
        (looking_at_robot ?h - id)
        ...
)

(:durative-action find_interactant
        :parameters (?i - interactant_id ?h - id)
        :duration ( = ?duration 0)
        :condition (and
                (at start (free_interactant_id ?i))
                (at start (looking_at_robot ?h)))
        :effect (and
                (at end (found_interactant ?i ?h))
                (at end (not (free_interactant_id ?i))))
)

...
```

This domain file contains only one action which makes planning a little pointless but this is only supposed to serve as an example how this action could be implemented. The types and predicates defined are only important for the planner which is why we will ignore them for now. The only thing to remember is their structure, e.g. `(found_interactant ?i - interactant_id ?h - id)` has two arguments and `interactant_id` and `id` which is the id of a detected human. We won't concern ourselves with how `interactant_id`s and `id`s are added to knowledge base here.

Looking at the action:

```pddl
(:durative-action find_interactant
        :parameters (?i - interactant_id ?h - id)
        :duration ( = ?duration 0)
        :condition (and
                (at start (free_interactant_id ?i))
                (at start (looking_at_robot ?h)))
        :effect (and
                (at end (found_interactant ?i ?h))
                (at end (not (free_interactant_id ?i))))
)
```

we start from the top:

* Our action is called `find_interactant` and has the following parameters

 ```pddl
:parameters (?i - interactant_id ?h - id)
 ```

 Hence, our action has too like this:

 ```
string interactant_id 
string id
---
pnp_msgs/ActionResult[] result
---

 ```

 _The names of the goal arguments could be `foo` and `bar` as well. Only the order of the arguments matters. Hence, the first argument will always the an `interactant_id` and the second will always be an `id`._

 The `result` has to be of type `pnp_msgs/ActionResult[]` as described in the `pnp_ros/README.md` file. This allows the pnp server to automatically update the KB once the action is finished.

* The duration if the action is set to 0 which means that it will run for as long as it takes. If the duration is set to a positive value, the action will always run for the specified duration either waiting after it was successful or being terminated once the time is up. This is automatically interpreted by the PNP generation and need not concern us here.
* Conditions are, like the duration, automatically interpreted by the PNP generation and will be checked before the action is executed. If any of the conditions is `false`, the plan will fail and replanning will be triggered.
* Effects describe the predicates that will be added or removed from the knowledgebase. _True predicates will be added, false predicates will be removed._ The effects are also automatically checked by the PNP before the action is started. If for some reason all the effects have come to pass already, the action will simply be skipped. After the action has been executed, the effects will be checked and if they have not come to pass, the action will be repeated. Hence, we have to make sure that either our action server sets the predicates accordingly or we have a separate component that updates the KB. The former will be described in the following, for the latter please have a look at the `pnp_servers/pepper_world_state_kb` package as an example.

### Writing your action server

For a more detailed explanaition of the `PluginServer`s please refer to the `pnp_ros/README.md`.

Our `find_interactant` action server could look something like this:

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer
from pepper_engage_human.msg import FindInteractantAction, FindInteractantResult
from pnp_msgs.msg import ActionResult


class FindInteractant(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self._ps = PNPSimplePluginServer(
            name=name,
            ActionSpec=FindInteractantAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._ps.start()
        rospy.loginfo("... done")

    def execute_cb(self, goal):
        rospy.loginfo("Finding interactant '%s'" % (goal.interactant_id,))

        # Here be dragons ...

        res = FindInteractantResult()
        res.result.append(ActionResult(cond="found_interactant__%s___%s" %(goal.interactant_id,goal.id), truth_value=True))
        res.result.append(ActionResult(cond="free_interactant_id__%s"%goal.interactant_id, truth_value=False))
        if self._ps.is_preempt_requested():
            self._ps.set_preempted()
        else:
            self._ps.set_succeeded(res)

if __name__ == "__main__":
    rospy.init_node("find_interactant")
    FindInteractant(rospy.get_name())
    rospy.spin()

```

Let's start from the top:

* The `PNPSimplePluginServer` is wrapper around the `SimpleActionServer` and has all the same functionalities plus it automatically registers your action with the running PNP server. It is initialised the exact same way as its vanilla counter part:

 ```python
        self._ps = PNPSimplePluginServer(
            name=name,
            ActionSpec=FindInteractantAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._ps.start()
 ```

 There also is a `PNPPluginServer` wrapping the standard `ActionServer`.

* The execute callback receives the goal we created earlier and does some magic processing. The most important bit is the result:

 ```python
        res = FindInteractantResult()
        res.result.append(ActionResult(cond="found_interactant__%s___%s" %(goal.interactant_id,goal.id), truth_value=True))
        res.result.append(ActionResult(cond="free_interactant_id__%s"%goal.interactant_id, truth_value=False))
        if self._ps.is_preempt_requested():
            self._ps.set_preempted()
        else:
            self._ps.set_succeeded(res)
 ```

 where `res.result` is just an array of `ActionResult`. These results are interpreted automatically and then added to the KB. Hence,

 * `ActionResult(cond="found_interactant__%s___%s" %(goal.interactant_id,goal.id), truth_value=True)` adds the predicate: `(found_interactant iid id)` with the values taken from the goal for `iid` and `id`, e.g. `(found_interactant iid_0 id_9523)`
 * `ActionResult(cond="free_interactant_id__%s"%goal.interactant_id, truth_value=False)` removes the predicate `(free_interactant_id iid)` form the KB using the values taken from the goal for `iid`, e.g. `(free_interactant_id iid_0)` is being removed.
 * The result is then passed on when the server completes successfully. The same can be done when he action is aborted. Only preemption is ignored.

* The name of the action server has to be the same as the name in the PDDL file:

 ```python
class FindInteractant(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self._ps = PNPSimplePluginServer(
            name=name,                          # <---
            ActionSpec=FindInteractantAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )

if __name__ == "__main__":
    rospy.init_node("find_interactant")         # <---
    FindInteractant(rospy.get_name())           # <---
    ...
 ```

 This way the correct action will be called. If the name is different, the action server will simply be ignored.


Concluding from all this, the main part of writing an action for the PNP execution framework is to replace your `SimpleActionServer` call with a `PNPSimplePluginServer`, make sure it returns the right values to update the KB, and to name it the same as your action in the PDDL file. The rest will be handled by the PNP server.