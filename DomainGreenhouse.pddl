(define (domain greenhouse-domain)
    (:requirements :strips :typing :fluents :durative-actions)
    
    (:types 
     robot plant pest waypoint arm
    )

    (:predicates
     (robot-at ?r - robot ?wp - waypoint)
     
     (pest-at ?p - pest ?wp - waypoint)
     (pest-sprayed ?p - pest)
     
     (plant-at ?pl - plant ?wp - waypoint)
     (plant-scanned ?pl - plant)
 
     (arm-base ?a - arm)
     (arm-scan ?a - arm)
     (arm-spray ?a - arm)
     (connected-wp ?from ?to - waypoint)
     (wp-visited ?wp - waypoint)
    )
    
    (:action move
      :parameters (?r - robot ?a - arm ?from - waypoint ?to - waypoint)
      :precondition (and
        (robot-at ?r ?from)
        (connected-wp ?from ?to)
        (arm-base ?a)
      )
      :effect (and
        (not (robot-at ?r ?from))
        (robot-at ?r ?to)
      )
    )
    
    (:durative-action spray
      :parameters (?r - robot ?p - pest ?wp - waypoint ?a - arm)
      :duration (= ?duration 3)
      :condition (and
        (at start (robot-at ?r ?wp))
        (at start (arm-base ?a))
        (at start (pest-at ?p ?wp))
        (over all (robot-at ?r ?wp))
      )
      :effect (and
        (at start (not (arm-base ?a)))
        (at start (arm-spray ?a))
        (at end (pest-sprayed ?p))
        (at end (not (arm-spray ?a)))
        (at end (arm-base ?a))
      )
    )
    
    (:durative-action scan
      :parameters (?r - robot ?pl - plant ?wp - waypoint ?a - arm)
      :duration (=?duration 3)
      :condition (and
        (at start (robot-at ?r ?wp))
        (at start (arm-base ?a))
        (at start (plant-at ?pl ?wp))
        (over all (robot-at ?r ?wp))
      )
      :effect (and
        (at start (not (arm-base ?a)))
        (at start (arm-scan ?a))
        
        (at end (not (arm-scan ?a)))
        (at end (arm-base ?a))
        (at end (plant-scanned ?pl))
      )
    )
)
    
    
    