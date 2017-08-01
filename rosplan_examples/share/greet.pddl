(define (domain pepper_demo)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
        distance
        text
        id
        interactant_id
        battery_level
        waypoint
        shop_id
)

(:predicates
        (human_exists ?h - id)
        (robot_distance ?h - id ?d - distance)
        (robot_at_home)
        (robot_at_waypoint ?w - waypoint)
        (robot_pose_unknown)
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
        (described_route ?s - shop_id ?w - waypoint)
        (finished_description ?s - shop_id ?w - waypoint)
        (wants_to_interact ?h - id)
        (heard ?t - text)
        (replied ?t - text)
        (keyword ?t - text)
)

(:durative-action finish_description
        :parameters (?s - shop_id ?w - waypoint)
        :duration ( = ?duration 0)
        :condition (and
                (at start (described_route ?s ?w)))
        :effect (and
                (at end (not (described_route ?s ?w)))
                (at end (finished_description ?s ?w)))
)

(:durative-action describe_route
        :parameters (?s - shop_id ?w - waypoint)
        :duration ( = ?duration 0)
        :condition (and
                (at start (robot_at_waypoint ?w)))
        :effect (and
                (at end (described_route ?s ?w))
                (at end (not (finished_description ?s ?w))))
)

(:durative-action move_to_waypoint
        :parameters (?t - waypoint ?f - waypoint)
        :duration ( = ?duration 0)
        :condition (and
                (at start (robot_at_waypoint ?f)))
        :effect (and
                (at end (robot_at_waypoint ?t))
                (at end (not (robot_at_waypoint ?f)))
                (at end (robot_pose_unknown)))
)

(:durative-action find_interactant
        :parameters (?i - interactant_id ?h - id)
        :duration ( = ?duration 0)
        :condition (and
                (at start (free_interactant_id ?i))
                (at start (wants_to_interact ?h)))
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
                (at start (said ?h ?t)))
        :effect (and
                (at end (engaged ?i ?t)))
)

(:durative-action engaged_by_human
        :parameters (?i - interactant_id ?t - text ?k - text)
        :duration ( = ?duration 0)
        :condition (and
                (at start (heard ?k))
                (at start (keyword ?k))
                (at start (replied ?t)))
        :effect (and
                (at end (engaged ?i ?t))
                (at end (not (free_interactant_id ?i))))
)

(:durative-action terminate_interaction
        :parameters (?i - interactant_id ?h - id ?t - text)
        :duration ( = ?duration 0)
        :condition (and
                (at start (found_interactant ?i ?h))
                (at start (robot_at_home)))
        :effect (and
                (at end (not (found_interactant ?i ?h)))
                (at end (free_interactant_id ?i))
                (at end (not (engaged ?i ?t)))
                (at end (not (human_exists ?h)))
                (at end (not (said ?h ?t))))
)

(:durative-action terminate_interaction_by_user
        :parameters (?i - interactant_id ?t - text ?k - text)
        :duration ( = ?duration 0)
        :condition (and
                (at start (heard ?k))
                (at start (keyword ?k))
                (at start (replied ?t))
                (at start (robot_at_home)))
        :effect (and
                (at end (free_interactant_id ?i))
                (at end (not (engaged ?i ?t)))
                (at end (not (heard ?k)))
                (at end (not (replied ?t))))
)

(:durative-action check_human_existance
        :parameters (?h - id ?i - interactant_id)
        :duration ( = ?duration 0)
        :effect (and
                (at end (human_exists ?h)))
)

(:durative-action goto
        :parameters (?h - id ?t - distance ?f - distance)
	:duration ( = ?duration 0)
        :condition (and
                (at start (human_exists ?h))
                (at start (robot_distance ?h ?f)))
        :effect (and
                (at end (robot_distance ?h ?t))
                (at start (not (robot_distance ?h ?f)))
                (at end (not (robot_at_home)))
                (at end (robot_pose_unknown)))
)

(:durative-action go_home
        :parameters ()
        :duration ( = ?duration 0)
        :condition (and
                (at start (robot_pose_unknown)))
        :effect (and
                (at end (robot_at_home))
                (at end (not (robot_pose_unknown))))
)
  
(:durative-action say
        :parameters (?h - id ?t - text ?d - distance)
        :duration ( = ?duration 0)
        :condition (and
                (at start (human_exists ?h))
                (at start (robot_distance ?h ?d))
                (at start (say_distance ?d))) 
	:effect (and
                (at end (said ?h ?t)))
)

(:durative-action respond
        :parameters (?t - text ?k - text)
        :duration ( = ?duration 0)
        :condition (and
                (at start (heard ?k))
                (at start (keyword ?k))) 
        :effect (and
                (at end (replied ?t)))
)
)
