namespace FSharpAHRS.Algorithm

module Madgwick = 
  open FSharpAHRS.Models
  open FSharpAHRS.Utils
  open System
  
  let internal recipSampleFreq sampleInterval = 
    let sampleFrequency = (1000. / sampleInterval)
    1.0 / sampleFrequency;
  let internal coordToList coord = [|coord.x; coord.y; coord.z|]
  let internal stateToTuple state = (state.q0, state.q1, state.q2, state.q3)
  let internal normalizeCoord coord = coord 
                                    |> coordToList 
                                    |> normScale 1.0
  let internal qDots (session:AHRSSession) gyro = 
    let qDot1 = 0.5 * (-session.State.q1 * gyro.x - session.State.q2 * gyro.y - session.State.q3 * gyro.z);
    let qDot2 = 0.5 * (session.State.q0 * gyro.x + session.State.q2 * gyro.z - session.State.q3 * gyro.y);
    let qDot3 = 0.5 * (session.State.q0 * gyro.y - session.State.q1 * gyro.z + session.State.q3 * gyro.x);
    let qDot4 = 0.5 * (session.State.q0 * gyro.z + session.State.q1 * gyro.y - session.State.q2 * gyro.x);
    (qDot1, qDot2, qDot3, qDot4)

  
  /// **Description**
  ///   Sets the session with updates state
  /// **Parameters**
  ///   * `session` - Old session `AHRSSession`
  ///   * `state` - New state `State`
  /// **Output Type**
  ///   * `AHRSSession` New session
  ///
  /// **Exceptions**
  ///
  let internal setSessionState (session:AHRSSession) [|q0;q1;q2;q3|] = 
        session.State.q0 <- q0;
        session.State.q1 <- q1;
        session.State.q2 <- q2;
        session.State.q3 <- q3;
        session


  
  /// **Description**
  ///   IMU algorithm update
  /// **Parameters**
  ///   * `session` - AHRS Session `AHRSSession`
  ///   * `accel` - accelerometer readings `Coordinate`
  ///   * `gyro` - gyroscope readings `Coordinate`
  ///   * `options` - Madwick options `MadgwickOpts`
  ///
  /// **Output Type**
  ///   * `AHRSSession` AHRS Session
  ///
  /// **Exceptions**
  ///
  let internal madgwickAHRSupdateIMU (session:AHRSSession) accel gyro options=
      let recipSampleFreq = recipSampleFreq options.sampleInterval
      let (q0, q1, q2, q3) = stateToTuple session.State
      let update = 
        let (qDot1,qDot2,qDot3,qDot4) = qDots session gyro 

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if (not (accel.x = 0.0 && accel.y = 0.0 && accel.z = 0.0)) then
          // Normalise accelerometer measurement
          let [|ax;ay;az|] = normalizeCoord accel

          // Auxiliary variables to avoid repeated arithmetic
          let [|v2q0;v2q1;v2q2;v2q3|] = scalarMultiply 2.0 [|q0; q1; q2; q3|]
          let [|v4q0;v4q1;v4q2|] = scalarMultiply 4.0 [|q0; q1; q2|]
          let [|v8q1;v8q2|] = scalarMultiply 8.0 [|q1; q2|]
          let [|q0q0;q1q1;q2q2;q3q3|] = square [|q0;q1; q2; q3|] 

          // Gradient decent algorithm corrective step
          let s0 = v4q0 * q2q2 + v2q2 * ax + v4q0 * q1q1 - v2q1 * ay
          let s1 = v4q1 * q3q3 - v2q3 * ax + 4.0 * q0q0 * q1 - v2q0 * ay - v4q1 + v8q1 * q1q1 + v8q1 * q2q2 + v4q1 * az
          let s2 = 4.0 * q0q0 * q2 + v2q0 * ax + v4q2 * q3q3 - v2q3 * ay - v4q2 + v8q2 * q1q1 + v8q2 * q2q2 + v4q2 * az
          let s3 = 4.0 * q1q1 * q3 - v2q1 * ax + 4.0 * q2q2 * q3 - v2q2 * ay

          subtract [|qDot1;qDot2;qDot3;qDot4|] (normScale options.beta [|s0;s1;s2;s3|]) //  Normalize and applys feedback
        else  
          [|qDot1;qDot2;qDot3;qDot4|]

      update
      |> scalarMultiply recipSampleFreq
      |> add [|q0;q1;q2;q3|]    // Integrate rate of change of quaternion to yield quaternion
      |> normScale 1.0           // Normalise quaternion
      |> setSessionState session

  
  /// **Description**
  ///   AHRS algorithm update
  /// **Parameters**
  ///   * `session` - AHRS Session `AHRSSession`
  ///   * `sensorInput` - Input of accelerometer, gyroscope, and magnetometer respectfully `SensorInput`
  ///   * `options` - Madwick options `MadgwickOpts`
  ///
  /// **Output Type**
  ///   * `AHRSSession` AHRS Session
  ///
  /// **Exceptions**
  ///
  let madgwickAHRSupdate (session:AHRSSession) {accel = accel; gyro = gyro; compass = compass;} options =
      let recipSampleFreq = recipSampleFreq options.sampleInterval
      let (q0, q1, q2, q3) = stateToTuple session.State
      let update =    
          let qDot1,qDot2,qDot3,qDot4 = qDots session gyro
          // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
          if (not (accel.x = 0.0 && accel.y = 0.0 && accel.z = 0.0)) then
            // Normalise accelerometer measurement
            let [|ax;ay;az|] = normalizeCoord accel

            // Normalise magnetometer measurement
            let [|mx;my;mz|] = normalizeCoord compass

            // Auxiliary variables to avoid repeated arithmetic
            let [|v2q0mx;v2q0my;v2q0mz;v2q1mx|] = scalarMultiply 2.0 (multiply [|q0; q0; q0; q1|] [|mx; my; mz; mx|])
            let [|v2q0;v2q1;v2q2;v2q3|] = scalarMultiply 2.0 [|q0; q1; q2; q3|]
            let [|q0q0;q1q1;q2q2;q3q3|] = square [|q0; q1; q2; q3|]
            let [|q0q1;q0q2;q0q3|] = scalarMultiply  q0 [|q1; q2; q3|]

            let v2q0q2 = 2.0 * q0 * q2
            let v2q2q3 = 2.0 * q2 * q3
            let q1q2 = q1 * q2
            let q1q3 = q1 * q3
            let q2q3 = q2 * q3

            // Reference direction of Earth's magnetic field
            let hx = mx * q0q0 - v2q0my * q3 + v2q0mz * q2 + mx * q1q1 + v2q1 * my * q2 + v2q1 * mz * q3 - mx * q2q2 - mx * q3q3
            let hy = v2q0mx * q3 + my * q0q0 - v2q0mz * q1 + v2q1mx * q2 - my * q1q1 + my * q2q2 + v2q2 * mz * q3 - my * q3q3
            let v2bx = Math.Sqrt (hx * hx + hy * hy)
            let v2bz = -v2q0mx * q2 + v2q0my * q1 + mz * q0q0 + v2q1mx * q3 - mz * q1q1 + v2q2 * my * q3 - mz * q2q2 + mz * q3q3
            let v4bx = 2.0 * v2bx
            let v4bz = 2.0 * v2bz

            // Gradient decent algorithm corrective step
            let s0 =
              -v2q2 * (2.0 * q1q3 - v2q0q2 - ax) +
              v2q1 * (2.0 * q0q1 + v2q2q3 - ay) -
              v2bz * q2 * (v2bx * (0.5 - q2q2 - q3q3) + v2bz * (q1q3 - q0q2) - mx) +
              (-v2bx * q3 + v2bz * q1) * (v2bx * (q1q2 - q0q3) + v2bz * (q0q1 + q2q3) - my) +
              v2bx * q2 * (v2bx * (q0q2 + q1q3) + v2bz * (0.5 - q1q1 - q2q2) - mz)
            let s1 =
              v2q3 * (2.0 * q1q3 - v2q0q2 - ax) +
              v2q0 * (2.0 * q0q1 + v2q2q3 - ay) -
              4.0 * q1 * (1. - 2.0 * q1q1 - 2.0 * q2q2 - az) +
              v2bz * q3 * (v2bx * (0.5 - q2q2 - q3q3) + v2bz * (q1q3 - q0q2) - mx) +
              (v2bx * q2 + v2bz * q0) * (v2bx * (q1q2 - q0q3) + v2bz * (q0q1 + q2q3) - my) +
              (v2bx * q3 - v4bz * q1) * (v2bx * (q0q2 + q1q3) + v2bz * (0.5 - q1q1 - q2q2) - mz)
            let s2 =
              -v2q0 * (2.0 * q1q3 - v2q0q2 - ax) +
              v2q3 * (2.0 * q0q1 + v2q2q3 - ay) -
              4.0 * q2 * (1. - 2.0 * q1q1 - 2.0 * q2q2 - az) +
              (-v4bx * q2 - v2bz * q0) * (v2bx * (0.5 - q2q2 - q3q3) + v2bz * (q1q3 - q0q2) - mx) +
              (v2bx * q1 + v2bz * q3) * (v2bx * (q1q2 - q0q3) + v2bz * (q0q1 + q2q3) - my) +
              (v2bx * q0 - v4bz * q2) * (v2bx * (q0q2 + q1q3) + v2bz * (0.5 - q1q1 - q2q2) - mz)
            let s3 =
              v2q1 * (2.0 * q1q3 - v2q0q2 - ax) +
              v2q2 * (2.0 * q0q1 + v2q2q3 - ay) +
              (-v4bx * q3 + v2bz * q1) * (v2bx * (0.5 - q2q2 - q3q3) + v2bz * (q1q3 - q0q2) - mx) +
              (-v2bx * q0 + v2bz * q2) * (v2bx * (q1q2 - q0q3) + v2bz * (q0q1 + q2q3) - my) +
              v2bx * q1 * (v2bx * (q0q2 + q1q3) + v2bz * (0.5 - q1q1 - q2q2) - mz)
            
            subtract [|qDot1;qDot2;qDot3;qDot4|] (normScale options.beta [|s0;s1;s2;s3|]) // Apply normalize and applys feedback
          else
            [|qDot1;qDot2;qDot3;qDot4|]
      

      // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
      if ((compass.x = 0. && compass.y = 0. && compass.z = 0.)) then
          madgwickAHRSupdateIMU session accel gyro options
      else
          // Rate of change of quaternion from gyroscope
        update
        |> scalarMultiply recipSampleFreq
        |> add [|q0;q1;q2;q3|]        // Integrate rate of change of quaternion to yield quaternion
        |> normScale 1.0               // Normalise quaternion
        |> setSessionState session


  /// **Description**
  ///   Converts state of AHRS euler to quaternion form
  /// **Parameters**
  ///   * `state` - state of AHRS `State`
  ///
  /// **Output Type**
  ///   * `Quaternion` state of AHRS in quaternion form
  ///
  /// **Exceptions**
  ///
  let quaternion state = 
    {x=state.q0; y=state.q1; z = state.q2; w = state.q3}


  /// **Description**
  ///   Converts state of AHRS euler to vector form
  /// **Parameters**
  ///   * `state` - state of AHRS `State`
  ///
  /// **Output Type**
  ///   * `Vector` state of AHRS in vector form
  ///
  /// **Exceptions**
  ///
  let vector state = 
    let q = quaternion state
    let angle = 2. * (Math.Acos q.w)
    let sinAngle = Math.Sin (angle / 2.)
    {x=q.x / sinAngle; y=q.y / sinAngle; z = q.z / sinAngle; angle = angle}


  /// **Description**
  ///   Converts state of AHRS to euler angle in radians
  /// **Parameters**
  ///   * `state` - state of AHRS `State`
  ///
  /// **Output Type**
  ///   * `Euler` state of AHRS in euler radians
  ///
  /// **Exceptions**
  ///
  let eulerAngles state = 
    let q = quaternion state
    let ww = q.w * q.w
    let xx = q.x * q.x
    let yy = q.y * q.y
    let zz = q.z * q.z
    {yaw = Math.Atan2(2. * (q.x * q.y + q.z * q.w), xx - yy - zz + ww)
     pitch = -Math.Asin(2. * (q.x * q.z - q.y * q.w));
     roll = Math.Atan2(2. * (q.y * q.z + q.x * q.w), -xx - yy + zz + ww)
    }


  /// **Description**
  ///   Converts state of AHRS to euler angle in degrees
  /// **Parameters**
  ///   * `state` - state of AHRS `State`
  ///
  /// **Output Type**
  ///   * `Euler` state of AHRS in euler degrees
  ///
  /// **Exceptions**
  ///
  let eulerAnglesDegrees state= 
    let getEulerAnglesRad = eulerAngles state
    {yaw = getEulerAnglesRad.yaw |> rad2deg; pitch = getEulerAnglesRad.pitch |> rad2deg; roll = getEulerAnglesRad.roll |> rad2deg}


  let rotationMatrix state = 
      let (q0, q1, q2, q3) = stateToTuple state
      [|1.0 - 2.0*q2*q2 - 2.0*q3*q3;
      2.0*q1*q2 - 2.0*q3*q0;
      2.0*q1*q3 + 2.0*q2*q0;
      2.0*q1*q2 + 2.0*q3*q0;
      1.0 - 2.0*q1*q1 - 2.0*q3*q3;
      2.0*q2*q3 - 2.0*q1*q0;
      2.0*q1*q3 - 2.0*q2*q0;
      2.0*q2*q3 + 2.0*q1*q0;
      1.0 - 2.0*q1*q1 - 2.0*q2*q2|]
    


  let applyDeclination declination acceleration = 
    let absNorthAcc = acceleration.North * Math.Cos(declination) + acceleration.East * Math.Sin(declination);
    let absEastAcc = acceleration.East * Math.Cos(declination) - acceleration.North * Math.Sin(declination);
    (absNorthAcc, absEastAcc)

    