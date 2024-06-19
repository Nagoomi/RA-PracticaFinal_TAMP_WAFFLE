(define (problem apilar-cubs-problema)
  (:domain apilar-cubs)
  (:objects
    cube1 cube2 cube3 cube4 cube5 cube6 - cub
    lloc1 lloc2 lloc3 lloc4 lloc5 lloc6 lloc7 lloc8 lloc9 - lloc
  )
  
  (:init
    (situat_a cube1 lloc1)
    (situat_a cube2 lloc2)
    (situat_a cube3 lloc3)
    (situat_a cube4 lloc4)
    (situat_a cube5 lloc5)
    (situat_a cube6 lloc6)
    (res_sobre cube1)
    (res_sobre cube2)
    (res_sobre cube3)
    (res_sobre cube4)
    (res_sobre cube5)
    (res_sobre cube6)
    (ma_buida)
    (adjacent lloc1 lloc2)
    (adjacent lloc2 lloc3)
    (adjacent lloc3 lloc4)
    (adjacent lloc4 lloc5)
    (adjacent lloc5 lloc6)
    (adjacent lloc6 lloc7)
    (adjacent lloc7 lloc8)
    (adjacent lloc8 lloc9)
  )
  
  (:goal
    (and
      (situat_a cube1 lloc7)
      (situat_a cube2 lloc8)
      (situat_a cube3 lloc9)
      (a_sobre cube4 cube2)
      (a_sobre cube5 cube3)
      (a_sobre cube6 cube5)
      (res_sobre cube1)
      (res_sobre cube4)
      (res_sobre cube6)
    )
  )
)




