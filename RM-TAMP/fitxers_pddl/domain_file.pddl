(define (domain tamp_cubes)

(:types object robot location)

(:predicates
       (handEmpty ?rob)
	   (holding ?rob ?obj)
       (in ?obj ?from))

(:action pick
:parameters (?rob - robot ?obj - obj ?from - location)
:precondition (and (handEmpty ?rob) (in ?obj ?from))
:effect (and (holding ?rob ?obj)
   (not (handEmpty ?rob)) ))

(:action place
:parameters (?rob - robot ?obj - objecte ?from - location)
:precondition (and (holding ?rob ?obj))
:effect (and (handEmpty ?rob) (in ?obj ?from)
   (not (holding ?rob ?obj)) ))
)