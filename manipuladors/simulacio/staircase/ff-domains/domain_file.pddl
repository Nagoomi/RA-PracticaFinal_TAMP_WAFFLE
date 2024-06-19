(define (domain staircase)

(:types cub robot loc)

(:predicates
       (handEmpty ?rob)
       (holding ?rob ?cub)
       (in ?cub ?from))

(:action pick
:parameters (?rob - robot ?cub - cub ?from - loc)
:precondition (and (handEmpty ?rob) (in ?cub ?from))
:effect (and (holding ?rob ?cub)
   (not (handEmpty ?rob)) ))

(:action place
:parameters (?rob - robot ?cub - cub ?from - loc)
:precondition (and (holding ?rob ?cub))
:effect (and (handEmpty ?rob) (in ?cub ?from)
   (not (holding ?rob ?cub)) ))
)
