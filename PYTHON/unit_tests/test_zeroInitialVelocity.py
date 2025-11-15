import numpy

from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.init_ks import init_ks
from vehiclemodels.init_st import init_st
from vehiclemodels.init_mb import init_mb
from vehiclemodels.vehicle_dynamics_ks import vehicle_dynamics_ks
from vehiclemodels.vehicle_dynamics_st import vehicle_dynamics_st
from vehiclemodels.vehicle_dynamics_mb import vehicle_dynamics_mb


def integrate_rk4(func, x0, u, p, dt, steps):
  x = numpy.array(x0, dtype=float)

  for _ in range(steps):
    k1 = numpy.array(func(x, u, p), dtype=float)
    k2 = numpy.array(func(x + 0.5 * dt * k1, u, p), dtype=float)
    k3 = numpy.array(func(x + 0.5 * dt * k2, u, p), dtype=float)
    k4 = numpy.array(func(x + dt * k3, u, p), dtype=float)

    x = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    if x.size >= 27:
      x[23:27] = numpy.maximum(x[23:27], 0.0)

  return x


def dyn_ks(x, u, p):
  return vehicle_dynamics_ks(x.tolist(), u, p)


def dyn_st(x, u, p):
  return vehicle_dynamics_st(x.tolist(), u, p)


def dyn_mb(x, u, p):
  return vehicle_dynamics_mb(x.tolist(), u, p)


def test_zeroInitialVelocity():
  # test_zeroInitialVelocity - unit_test_function for starting with zero 
  # initial velocity
  #
  # Some vehicle models have a singularity when the vehicle is not moving.
  # This test checks whether the vehicles can handle this situation
  #
  # Syntax:  
  #    res = test_zeroInitialVelocity()
  #
  # Inputs:
  #    ---
  #
  # Outputs:
  #    res - boolean result (0/empty: test not passed, 1: test passed)
  #
  # Example: 
  #
  # Other m-files required: none
  # Subfunctions: none
  # MAT-files required: none
  #
  # See also: ---

  # Author:       Matthias Althoff
  # Written:      16-December-2017
  # Last update:  ---
  # Last revision:---

  #------------- BEGIN CODE --------------

  # initialize result
  res = []

  # load parameters
  p = parameters_vehicle2()
  g = 9.81  #[m/s^2]

  # set options --------------------------------------------------------------
  tStart = 0  #start time
  tFinal = 1  #start time

  delta0 = 0
  vel0 = 0
  Psi0 = 0
  dotPsi0 = 0
  beta0 = 0
  sy0 = 0
  initialState = [0,sy0,delta0,vel0,Psi0,dotPsi0,beta0]  #initial state for simulation
  x0_KS = init_ks(initialState)  #initial state for kinematic single-track model
  x0_ST = init_st(initialState)  #initial state for single-track model
  x0_MB = init_mb(initialState, p)  #initial state for multi-body model
  #--------------------------------------------------------------------------

  dt = 1e-4
  steps = int((tFinal - tStart) / dt)

  # set input: rolling car (velocity should stay constant)
  u = [0, 0]

  # simulate multi-body model
  x_roll = integrate_rk4(dyn_mb, x0_MB, u, p, dt, steps)

  # simulate single-track model
  x_roll_st = integrate_rk4(dyn_st, x0_ST, u, p, dt, steps)

  # simulate kinematic single-track model
  x_roll_ks = integrate_rk4(dyn_ks, x0_KS, u, p, dt, steps)

  # check correctness
  # ground truth
  x_roll_gt = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.9396925632701572e-18, 1.0382619602630607e-18, -0.001383413646355998, -0.0020336342166179544, -6.8100035962383285e-18, 0.017624852018447854, 0.007165545259572071, 2.499120748142407e-18, -1.4334324855610345e-18, -7.3404080848722666e-18, 0.018676373794485092, 0.00037525175879187217, 1.2669696045979021e-18, -2.8465346036555629e-18, -7.3631025851755168e-18, 0.015462186524627917, 0.00002516250917098079, 1.7110213358664119, 1.7110213358664119, 0.0, 0.0, 1.1218513664139043e-18, 1.4834796531562799e-18]
  
  #comparison
  res.append(numpy.all(numpy.abs(x_roll - x_roll_gt) < 1e-2))
  res.append(numpy.allclose(x_roll_st, x0_ST, atol=1e-12))
  res.append(numpy.allclose(x_roll_ks, x0_KS, atol=1e-12))
  #--------------------------------------------------------------------------

  # set input: decelerating car ---------------------------------------------
  v_delta = 0
  acc = -0.7*g
  u = [v_delta, acc]

  # simulate multi-body model
  x_dec = integrate_rk4(dyn_mb, x0_MB, u, p, dt, steps)

  # simulate single-track model
  x_dec_st = integrate_rk4(dyn_st, x0_ST, u, p, dt, steps)

  # simulate kinematic single-track model
  x_dec_ks = integrate_rk4(dyn_ks, x0_KS, u, p, dt, steps)

  # check correctness
  #ground truth
  x_dec_gt = [1.8225618646966624, -0.0086273097089525977, 0.0, -3.4554290247963668, 0.00047929531592577661, -1.5920968922503272e-05, -0.00025302742748178734, -0.00011973072674828772, -0.018779685455296551, -0.0090084648185682851, 0.017283043089364594, 0.018135231336119616, 0.0086135837859677312, -4.481562597218733e-05, -1.4272890015767374e-05, 0.017349174844723529, 0.021532679755773425, 0.00038187032873207187, -3.3920811271631363e-05, -9.4784375662071155e-06, 0.017377021416567273, 0.012706419736429759, 9.4015391546569673e-06, 0.0, 0.0, 0.0, 0.0, -0.00020661575881509941, -0.00017869393157893184]  # ground truth for multi-body model
  x_dec_st_gt = [-3.4335000000000013, 0.0000000000000000, 0.0000000000000000, -6.8670000000000018, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000]  # ground truth for single-track model
  x_dec_ks_gt = [-3.4335000000000013, 0.0000000000000000, 0.0000000000000000, -6.8670000000000018, 0.0000000000000000]  # ground truth for kinematic single-track model
  
  #compare
  res.append(numpy.all(numpy.abs(x_dec - x_dec_gt) < 1e-2))
  res.append(numpy.all(numpy.abs(x_dec_st - x_dec_st_gt) < 1e-2))
  res.append(numpy.all(numpy.abs(x_dec_ks - x_dec_ks_gt) < 1e-2))
  #--------------------------------------------------------------------------


  # set input: accelerating car (wheel spin and velocity should increase more wheel spin at rear)
  v_delta = 0.15
  acc = 0.63*g
  u = [v_delta, acc]

  # simulate multi-body model
  x_acc = integrate_rk4(dyn_mb, x0_MB, u, p, dt, steps)

  # simulate single-track model
  x_acc_st = integrate_rk4(dyn_st, x0_ST, u, p, dt, steps)

  # simulate kinematic single-track model
  x_acc_ks = integrate_rk4(dyn_ks, x0_KS, u, p, dt, steps)

  # check correctness
  #ground truth
  x_acc_gt = [1.6871562929572532, 0.0042854001980273818, 0.14999999999998667, 3.1974132719678994, 0.33806720788220607, 0.89130846925333573, -0.018585793734759059, -0.05569523922934138, 0.014166803684133662, 0.010814605916134869, -0.62911782106847891, 0.017269790314292637, 0.0025945395929668643, -0.0042181329490571275, -0.011573377836930512, 0.45274405656671479, 0.016136574608443133, -0.0012368703361228014, -0.002363031148676171, -0.0072193161379929842, -1.8638044064961889, 0.017959249779534586, 0.0010263382407978082, 11.132619776576281, 7.5830653080368968, 308.32216462068976, 310.90017338219224, -0.019679020182265954, -0.0083587135435600583]
  x_acc_st_gt = [3.0731976046859715, 0.2869835398304389, 0.1500000000000000, 6.1802999999999999, 0.1097747074946325, 0.3248268063223301, 0.0697547542798040]  # ground truth for single-track model
  x_acc_ks_gt = [3.0845676868494927, 0.1484249221523042, 0.1500000000000000, 6.1803000000000017, 0.1203664469224163]  # ground truth for kinematic single-track model
    
  #compare
  res.append(numpy.all(numpy.abs(x_acc - x_acc_gt) < 1e-2))
  res.append(numpy.all(numpy.abs(x_acc_st - x_acc_st_gt) < 1e-2))
  res.append(numpy.all(numpy.abs(x_acc_ks - x_acc_ks_gt) < 1e-2))
  #--------------------------------------------------------------------------


  # steering to left---------------------------------------------------------
  v_delta = 0.15
  u = [v_delta, 0]

  # simulate multi-body model
  x_left = integrate_rk4(dyn_mb, x0_MB, u, p, dt, steps)

  # simulate single-track model
  x_left_st = integrate_rk4(dyn_st, x0_ST, u, p, dt, steps)

  # simulate kinematic single-track model
  x_left_ks = integrate_rk4(dyn_ks, x0_KS, u, p, dt, steps)

  # check correctness
  x_left_gt = [0.0, 0.0, 0.14999999999998667, 0.0, 0.0, 0.0, 0.00030874729123515774, 0.011699177588952989, -0.0013796623269185146, -0.0019195672722097301, -0.0066065475420350496, 0.017624825341632389, 0.007164007130114107, 0.00015051447265641363, 0.0091936447322982333, -0.017987682527725091, 0.018675048631672228, 0.00035569258254374372, 0.00016978794795261313, 0.0016007735062562938, -0.01772584741880711, 0.015463728064918913, 4.9390231495498064e-05, 1.7055154881400041, 1.7163126649152141, 0.0, 0.0, 0.00023660612929119265, 0.00037315260622417255]
  x_left_st_gt = [0.0, 0.0, 0.14999999999998667, 0.0, 0.0, 0.0, 0.083374602676255474]  # ground truth for single-track model
  x_left_ks_gt = [0.0, 0.0, 0.14999999999998667, 0.0, 0.0]  # ground truth for kinematic single-track model
  
  res.append(numpy.all(numpy.abs(x_left - x_left_gt) < 1e-2))
  res.append(numpy.all(numpy.abs(x_left_st - x_left_st_gt) < 1e-2))
  res.append(numpy.all(numpy.abs(x_left_ks - x_left_ks_gt) < 1e-2))
  #--------------------------------------------------------------------------

  # obtain final result
  res = all(res)
  print(res)
  
  return res

  #------------- END OF CODE --------------
