(define (domain pepper_demo)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
        distance
        text
        id
        interactant_id
        battery_level
)

(:predicates
        (human_exists ?h - id)
        (robot_distance ?h - id ?d - distance)
        (say_distance ?d - distance)
        (said ?h - id ?t - text)
        (found_interactant ?i - interactant_id ?h - id)
        (engaged ?i - interactant_id ?t - text)
        (free_interactant_id ?i - interactant_id)
        (face_detected ?h - id)
        (is_waving ?h - id)
        (looking_at_robot ?h - id)
        (hatch_closed)
        (hatch_open)
        (charging)
        (battery ?b - battery_level)
        (tracking ?h - id)
        (no_tracking)
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

(:durative-action start_tracking_person
        :parameters (?h - id ?i - interactant_id)
        :duration ( = ?duration 0)
        :condition (and
                (at start (found_interactant ?i ?h))
                (at start (no_tracking)))
        :effect (and
                (at end (tracking ?h))
                (at end (not (no_tracking))))
)

(:durative-action stop_tracking_person
        :parameters (?h - id ?i - interactant_id)
        :duration ( = ?duration 0)
        :condition (and
                (at start (tracking ?h)))
        :effect (and
                (at end (not (tracking ?h)))
                (at end (no_tracking)))
)

(:durative-action engage_human
        :parameters (?i - interactant_id ?h - id ?t - text)
        :duration ( = ?duration 0)
        :condition (and
                (at start (found_interactant ?i ?h))
                (at start (tracking ?h))
                (at start (said ?h ?t)))
        :effect (and
                (at end (engaged ?i ?t)))
)

(:durative-action terminate_interaction
        :parameters (?i - interactant_id ?h - id ?t - text)
        :duration ( = ?duration 0)
        :condition (and
                (at start (no_tracking))
                (at start (found_interactant ?i ?h)))
        :effect (and
                (at end (not (found_interactant ?i ?h)))
                (at end (free_interactant_id ?i))
                (at end (not (engaged ?i ?t)))
                (at end (not (human_exists ?h))))
)

(:durative-action check_human_existance
        :parameters (?h - id)
        :duration ( = ?duration 0)
        :effect (and
                (at end (human_exists ?h)))
)

(:durative-action goto
        :parameters (?h - id ?t - distance ?f - distance)
	:duration ( = ?duration 0)
        :condition (and
                (at start (no_tracking))
                (at start (human_exists ?h))
                (at start (robot_distance ?h ?f)))
        :effect (and
                (at end (robot_distance ?h ?t))
                (at start (not (robot_distance ?h ?f))))
)
  
(:durative-action say
        :parameters (?h - id ?t - text ?d - distance)
        :duration ( = ?duration 5)
        :condition (and
                (at start (human_exists ?h))
                (at start (tracking ?h))
                (over all (robot_distance ?h ?d))
                (at start (say_distance ?d))) 
	:effect (and
                (at end (said ?h ?t)))
)
)
