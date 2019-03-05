module FSharpAHRS.UnitTests

open NUnit.Framework
open Swensen.Unquote
open Newtonsoft.Json
open System.IO
open FSharpAHRS.Models
open FSharpAHRS.Algorithm.Madgwick

let sampleInterval = 10.;
let opts = {beta = 0.4; sampleInterval = 20.}
let readFileAsJson (filePath: string) = File.ReadAllText filePath
let sensorInputs path = JsonConvert.DeserializeObject<SensorInput list>(readFileAsJson path)
let round (num : float) =  System.Math.Round (num, 4)
let assertVec (vector1:Vector) (vector2:Vector) = {x = round vector1.x ;y = round vector1.y;z = round vector1.z;angle = round vector1.angle} =! {x = round vector2.x ;y = round vector2.y;z = round vector2.z;angle = round vector2.angle} 
let assertState (state1:State) (state2:State) = {q0 = round state1.q0 ;q1 = round state1.q1;q2 = round state1.q2;q3 = round state1.q3} =! {q0 = round state2.q0 ;q1 = round state2.q1;q2 = round state2.q2;q3 = round state2.q3} 
let assertEuler (euler1:Euler) (euler2:Euler) = {pitch = round euler1.pitch ;roll = round euler1.roll;yaw = round euler1.yaw} =! {pitch = round euler2.pitch ;roll = round euler2.roll;yaw = round euler2.yaw}


module ``AHRS::madgwickAHRSupdate`` =

    [<Test>]
    let ``AHRS::madgwickAHRSupdate basic test`` () =
        let session = AHRSSession()

        let solution = { q0 =  0.9977785200604344;
                         q1 = -0.06527815857362336;
                         q2 = -0.0070671609098166685;
                         q3 = 0.011262422293675977 }
        let session = sensorInputs "../../../data/RandomIMU.json" 
                      |> List.map(fun x -> madgwickAHRSupdate session x opts)
                      |> List.last
        assertState session.State solution
    
    [<Test>]
    let ``AHRS::madgwickAHRSupdate zero accel`` () =
        let session = AHRSSession()

        let solution = { q0= 0.9986469757547656;
                         q1= -0.05004505030636578;
                         q2= -0.010953594264511663;
                         q3= 0.008929139285652693 }
        let session = sensorInputs "../../../data/RandomIMUZeroAccel.json" 
                      |> List.map(fun x -> madgwickAHRSupdate session x opts)
                      |> List.last

        assertState session.State solution


    [<Test>]
    let ``AHRS::madgwickAHRSupdate zero compass`` () =
        let session = AHRSSession()

        let solution = { q0 =  0.9977227854634741;
                         q1 =  -0.06592353515908193;
                         q2 =  -0.010991609531669272;
                         q3 =  0.009083798711928794 }
        let session = sensorInputs "../../../data/RandomIMUZeroCompass.json" 
                      |> List.map(fun x -> madgwickAHRSupdate session x opts)
                      |> List.last

        assertState session.State solution

    [<Test>]
    let ``AHRS::madgwickAHRSupdate zero accel and compass`` () =
        let session = AHRSSession()

        let solution = { q0= 0.9986469757547656;
                         q1= -0.05004505030636578;
                         q2= -0.010953594264511663;
                         q3= 0.008929139285652693 }
        let session = sensorInputs "../../../data/RandomIMUZeroAccel&Compass.json" 
                      |> List.map(fun x -> madgwickAHRSupdate session x opts)
                      |> List.last

        assertState session.State solution

module ``AHRS::quaternion`` =

    [<Test>]
    let ``AHRS::quaternion basic test`` () =

        let state = { q0 =  0.9977785200604344;
                         q1 = -0.06527815857362336;
                         q2 = -0.0070671609098166685;
                         q3 = 0.011262422293675977 }
        let quaternion = quaternion state

        quaternion =! { x =  0.9977785200604344;
                         y = -0.06527815857362336;
                         z = -0.0070671609098166685;
                         w = 0.011262422293675977 }

module ``AHRS::vector`` =

    [<Test>]
    let ``AHRS::vector basic test`` () =

        let state = { q0 =  0.9977785200604344;
                         q1 = -0.06527815857362336;
                         q2 = -0.0070671609098166685;
                         q3 = 0.011262422293675977 }
        let vector = vector state

        assertVec vector { angle= 3.119067332791951;
                           x= 0.9978418062703257;
                           y= -0.06528229897869497;
                           z= -0.007067609159422851 }

module ``AHRS::eulerAngles`` =

    [<Test>]
    let ``AHRS::eulerAngles basic test`` () =

        let state = { q0 =  0.9977785200604344;
                         q1 = -0.06527815857362336;
                         q2 = -0.0070671609098166685;
                         q3 = 0.011262422293675977 }
        let eulerAngles = eulerAngles state

        assertEuler eulerAngles {yaw = -0.13080860653252974;
                                 pitch= 0.012632878340449873;
                                 roll= 3.1181911820426786 }

module ``AHRS::eulerAnglesDegrees`` =

    [<Test>]
    let ``AHRS::eulerAngles basic test`` () =

        let state = { q0 =  0.9977785200604344;
                         q1 = -0.06527815857362336;
                         q2 = -0.0070671609098166685;
                         q3 = 0.011262422293675977 }
        let eulerAnglesDegrees = eulerAnglesDegrees state

        assertEuler eulerAnglesDegrees { yaw= -7.494781078301364;
                                         pitch= 0.7238106120100093;
                                         roll= 178.65919444595485 }
