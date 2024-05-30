(define (domain tamp_cubes)

(:types object robot location)

(:predicates
       (handEmpty ?rob - robot)
       (holding ?rob - robot ?obj - object)
       (in ?obj - object ?from - location))

(:action pick
  :parameters (?rob - robot ?obj - object ?from - location)
  :precondition (and (handEmpty ?rob) (in ?obj ?from))
  :effect (and (holding ?rob ?obj)
               (not (handEmpty ?rob))
               (not (in ?obj ?from))))

(:action place
  :parameters (?rob - robot ?obj - object ?to - location)
  :precondition (holding ?rob ?obj)
  :effect (and (handEmpty ?rob) 
               (in ?obj ?to)
               (not (holding ?rob ?obj))))
)

