import { accelerationConstraints, steeringConstraints } from './constraints.js';
import { VehicleParameters } from './types.js';

export function vehicleDynamicsKsCog(x: number[], uInit: number[], params: VehicleParameters): number[] {
  if (x.length !== 5 || uInit.length !== 2) {
    throw new Error('vehicleDynamicsKsCog expects state length 5 and control length 2');
  }

  const lWb = params.a + params.b;

  const u = [
    steeringConstraints(x[2], uInit[0], params.steering),
    accelerationConstraints(x[3], uInit[1], params.longitudinal),
  ];

  const beta = Math.atan(Math.tan(x[2]) * params.b / lWb);

  const f = new Array<number>(5);
  f[0] = x[3] * Math.cos(beta + x[4]);
  f[1] = x[3] * Math.sin(beta + x[4]);
  f[2] = u[0];
  f[3] = u[1];
  f[4] = x[3] * Math.cos(beta) * Math.tan(x[2]) / lWb;
  return f;
}
