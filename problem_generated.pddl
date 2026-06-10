(define (problem greenhouse-generated-problem)
  (:domain greenhouse-domain)

  (:objects
    robot1 - robot
        arm1 - arm
        manual_start_wp wp_16 wp_17 wp_18 wp_19 - waypoint
  )

  (:init
    (robot-at robot1 wp_18)
        (wp-visited manual_start_wp)
        (wp-visited wp_16)
        (wp-visited wp_17)
        (wp-visited wp_18)
        (wp-scanned manual_start_wp)
        (wp-scanned wp_16)
        (wp-scanned wp_17)
        (wp-scanned wp_18)
        (arm-base arm1)
        (connected-wp manual_start_wp wp_16)
        (connected-wp manual_start_wp wp_17)
        (connected-wp wp_16 wp_17)
        (connected-wp wp_16 wp_18)
        (connected-wp wp_16 wp_19)
        (connected-wp wp_17 wp_16)
        (connected-wp wp_17 wp_18)
        (connected-wp wp_17 wp_19)
        (connected-wp wp_18 wp_16)
        (connected-wp wp_18 wp_17)
        (connected-wp wp_19 wp_16)
        (connected-wp wp_19 wp_17)
        (connected-wp wp_19 wp_18)
  )

  (:goal
    (and
      (wp-visited wp_19)
          (wp-scanned wp_19)
    )
  )
)
