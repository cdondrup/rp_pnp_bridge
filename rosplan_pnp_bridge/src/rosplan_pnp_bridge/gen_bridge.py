# -*- coding: utf-8 -*-
"""
Created on Thu Aug 18 15:25:06 2016

@author: cd32
"""
import rospy
from rosplan_knowledge_msgs.srv import GetDomainOperatorDetailsService
import pnpgen_ros.utils as ut
from pnpgen_ros.pnpgen_bridge_abstractclass import PNPGenBridgeAbstractclass
import rosplan_pnp_bridge.rosplan_feedback_server as kb


class GenBridge(PNPGenBridgeAbstractclass):
    def __init__(self):
        super(GenBridge, self).__init__()

    def new_rosplan_action(self, name, duration, parameters, id):
        return [self.new_action_list(
                    actions=[self.new_action(
                        name=kb.ACTION_ID,
                        parameters=[kb.START, str(id)]
                    )]
                ), self.new_action_list(
                    actions=[self.new_action(
                        name=str(name),
                        duration=int(duration),
                        parameters=parameters.values()
                    )]
                ), self.new_action_list(
                    actions=[self.new_action(
                        name=kb.ACTION_ID,
                        parameters=[kb.END, str(id)]
                    )]
                )]

    def parse_plan_msg(self, msg):
        plan = self.new_plan()

        for action in msg.plan:
            parameters = {str(p.key):str(p.value) for p in action.parameters}
            name = str(action.name)
            plan.actions.extend(
                self.new_rosplan_action(
                    name=name,
                    duration=action.duration,
                    parameters=parameters,
                    id=action.action_id
                )
            )

            op = self.get_conditions_and_effects(name)
            add = ut.create_predicate(
                parameters,
                op.at_start_add_effects,
                op.at_end_add_effects
            )
            dele = ut.create_predicate(
                parameters,
                op.at_start_del_effects,
                op.at_end_del_effects
            )
            cond = ut.create_predicate(
                parameters,
                op.at_start_simple_condition,
                op.at_end_simple_condition
            )

            # Fail plan if conditions are false
            plan.execution_rules.pnp_execution_rule_array.append(
                self.new_execution_rule(
                    timing=PNPGenBridgeAbstractclass.BEFORE,
                    action_name=name,
                    condition=ut.create_condition("not", ut.create_condition("and",cond)),
                    recovery=self.new_action_list(
                        actions=self.fail_plan()
                    )
                )
            )

            # Skip action if all the effects have happened already
            plan.execution_rules.pnp_execution_rule_array.append(
                self.new_execution_rule(
                    timing=PNPGenBridgeAbstractclass.BEFORE,
                    action_name=name,
                    condition=ut.create_condition("and", add, ut.create_condition("not", dele)),
                    recovery=self.new_action_list(
                        actions=self.skip_action()
                    )
                )
            )

            # Fail plan if action fails
            plan.execution_rules.pnp_execution_rule_array.append(
                self.new_execution_rule(
                    timing=PNPGenBridgeAbstractclass.DURING,
                    action_name=name,
                    condition="action_failed",
                    recovery=self.new_action_list(
                        actions=self.fail_plan()
                    )
                )
            )

            # Restart action if effects haven't happened
            plan.execution_rules.pnp_execution_rule_array.append(
                self.new_execution_rule(
                    timing=PNPGenBridgeAbstractclass.AFTER,
                    action_name=name,
                    condition=ut.create_condition("and", dele, ut.create_condition("not", add)),
                    recovery=self.new_action_list(
                        actions=self.restart_action()
                    )
                )
            )

        return plan

    def get_conditions_and_effects(self, action_name):
        while not rospy.is_shutdown():
            try:
                s = rospy.ServiceProxy(
                    "/kcl_rosplan/get_domain_operator_details",
                    GetDomainOperatorDetailsService
                )
                s.wait_for_service(timeout=1.)
            except rospy.ROSException:
                rospy.logwarn("Could not communicate with '%s' service. Retrying in 1 second." % "/kcl_rosplan/get_domain_operator_details")
                rospy.sleep(1.)
            else:
                try:
                    r = s(action_name)
                except rospy.ROSInterruptException:
                    return
                else:
                    return r.op
