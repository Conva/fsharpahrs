namespace FSharpAHRS

module Utils = 
    let internal rad2deg num = num * (180.0 / System.Math.PI)
    let internal deg2rad num = num * (System.Math.PI / 180.0)
    let inline internal invSqrt x = x ** -0.5 
    let inline internal add arr1 arr2= Array.map2 (+) arr1 arr2
    let inline internal subtract arr1 arr2= Array.map2 (-) arr1 arr2
    let inline internal multiply arr1 arr2= Array.map2 (*) arr1 arr2
    let inline internal scalarMultiply (s : float) arr1 = Array.map (fun elem -> elem * s) arr1
    let inline internal square arr1 = multiply arr1 arr1
    let inline internal normFactor arr1 = (square arr1) |> Array.sum |> invSqrt
    let inline internal normScale factor arr1= scalarMultiply (factor*(arr1 |> normFactor)) arr1
