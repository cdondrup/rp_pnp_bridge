(define (domain pepper_demo)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
        distance
        text
)

(:predicates
        (robot_at ?d - distance)
        (say_at ?d - distance)
        (said ?t - text)
)

(:durative-action goto
        :parameters (?t - distance ?f - distance)
	:duration ( = ?duration 5)
        :condition (and
                (at start (robot_at ?f)))
        :effect (and
                (at end (robot_at ?t))
                (at start (not (robot_at ?f))))
)
  
(:durative-action say
        :parameters (?t - text ?d - distance)
        :duration ( = ?duration 3)
        :condition (and
                (at start (robot_at ?d))
                (at start (say_at ?d))) 
	:effect (and
                (at end (said ?t)))
)
)
