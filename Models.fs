namespace FSharpAHRS

module Models = 

    type MadgwickOpts = {
      beta : float
      sampleInterval : float
    }

    type Quaternion = {
        x : float
        y : float
        z : float
        w : float
    }

    type Vector = {
        x : float
        y : float
        z : float
        angle : float
    }

    type Euler = {
        pitch : float
        roll : float
        yaw : float
    }

    type Coordinate = {
      x : float
      y : float
      z : float
    }

    type Acceleration = {
      North : float
      East : float
      Up : float
    }


    type SensorInput = {
        accel : Coordinate
        gyro: Coordinate
        compass : Coordinate
    }


    type State = {
        mutable q0 : float
        mutable q1 : float
        mutable q2 : float
        mutable q3 : float
    }

    type AHRSSession() = 
      let _state = {q0 = 1.; q1 = 0.; q2 = 0.; q3 = 0.}
      member this.State with get() = _state