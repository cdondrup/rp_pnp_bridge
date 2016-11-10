(define (domain pepper_demo)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
        distance
        text
        id
)

(:predicates
        (human_exists ?h - id)
        (robot_distance ?h - id ?d - distance)
        (say_distance ?d - distance)
        (said ?h - id ?t - text)
)

(:durative-action check_human_existance
        :parameters (?h - id)
        :duration ( = ?duration 0)
        :effect (and
                (at end (human_exists ?h)))
)

(:durative-action goto
        :parameters (?h - id ?t - distance ?f - distance)
	:duration ( = ?duration 5)
        :condition (and
                (at start (human_exists ?h))
                (at start (robot_distance ?h ?f)))
        :effect (and
                (at end (robot_distance ?h ?t))
                (at end (not(human_exists ?h)))
                (at start (not (robot_distance ?h ?f))))
)
  
(:durative-action say
        :parameters (?h - id ?t - text ?d - distance)
        :duration ( = ?duration 3)
        :condition (and
                (at start (human_exists ?h))
                (at start (robot_distance ?h ?d))
                (at start (say_distance ?d))) 
	:effect (and
                (at end (said ?h ?t)))
)
)
