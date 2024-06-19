(define (domain apilar-cubs)
  (:requirements :strips :typing)
  (:types cub lloc)
  
  (:predicates
    (res_sobre ?x - cub)
    (a_taula ?x - cub)
    (a_sobre ?x - cub ?y - cub)
    (agafant ?x - cub)
    (ma_buida)
    (situat_a ?c - cub ?l - lloc)
    (adjacent ?l1 - lloc ?l2 - lloc)
  )

  (:action agafar
    :parameters (?c - cub ?l - lloc)
    :precondition (and (situat_a ?c ?l) (res_sobre ?c) (ma_buida))
    :effect (and (not (situat_a ?c ?l)) (not (res_sobre ?c)) (not (ma_buida)) (agafant ?c))
  )

  (:action deixar
    :parameters (?c - cub ?l - lloc)
    :precondition (and (agafant ?c))
    :effect (and (situat_a ?c ?l) (not (agafant ?c)) (res_sobre ?c) (ma_buida) (a_taula ?c))
  )

  (:action apilar
    :parameters (?c - cub ?base - cub)
    :precondition (and (agafant ?c) (res_sobre ?base))
    :effect (and (a_sobre ?c ?base) (not (agafant ?c)) (res_sobre ?c) (not (res_sobre ?base)) (ma_buida))
  )
)
