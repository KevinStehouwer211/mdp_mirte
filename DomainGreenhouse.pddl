(define (domain greenhouse-domain)
  (:requirements :strips :typing :fluents :durative-actions)

  (:types
    robot plant pest waypoint arm
  )

  (:predicates
    (robot-at ?r - robot ?wp - waypoint)

    (pest-at ?p - pest ?wp - waypoint)
    (pest-sprayed ?p - pest)

    (arm-base ?a - arm)
    (arm-scan ?a - arm)
    (arm-spray ?a - arm)

    (connected-wp ?from ?to - waypoint)
    (scan-connected-wp ?from ?to - waypoint)
    (bin-waypoint-start ?wp - waypoint)
    (bin-waypoint-end ?wp - waypoint)

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
      (wp-visited ?to)
    )
  )

  (:action start-scan
    :parameters (?r - robot ?a - arm ?wp - waypoint)
    :precondition (and
      (robot-at ?r ?wp)
      (arm-base ?a)
      (bin-waypoint-start ?wp)
    )
    :effect (and
      (not (arm-base ?a))
      (arm-scan ?a)
      (wp-scanned ?wp)
    )
  )

  (:action move-while-scan
    :parameters (?r - robot ?a - arm ?from - waypoint ?to - waypoint)
    :precondition (and
      (robot-at ?r ?from)
      (scan-connected-wp ?from ?to)
      (arm-scan ?a)
    )
    :effect (and
      (not (robot-at ?r ?from))
      (robot-at ?r ?to)
      (wp-visited ?to)
      (wp-scanned ?to)
    )
  )

  (:action finish-scan
    :parameters (?r - robot ?a - arm ?wp - waypoint)
    :precondition (and
      (robot-at ?r ?wp)
      (arm-scan ?a)
      (bin-waypoint-end ?wp)
    )
    :effect (and
      (not (arm-scan ?a))
      (arm-base ?a)
    )
  )
)

  
