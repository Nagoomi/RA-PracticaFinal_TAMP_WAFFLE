(define (problem basic_problem)
    (:domain tamp_cubes)
    (:objects
        pos1 pos2 pos3 pos4 pos5 pos6 - location
        cube1 cube2 cube3 - object
        ur3a - robot
    )
    (:init
        (in cube1 pos1)
        (in cube2 pos2)
        (in cube3 pos3)
        (handEmpty ur3a)
    )
    (:goal
        (and
            (in cube1 pos4)
            (in cube2 pos5)
            (in cube3 pos6)
        )
    )
)






