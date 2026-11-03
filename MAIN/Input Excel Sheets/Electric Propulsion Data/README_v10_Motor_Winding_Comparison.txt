Albatross v10 motor-winding and 12S battery comparison

MotorCandidates worksheet: five NeuMotors 6530 windings, with actual parameters from manufacturer's Oct 2023 65xx sheet:
https://neumotors.com/wp-content/uploads/2023/10/2023-10-Oct-11-neumotors-65xx-datasheet.pdf
Imax is the motor saturation-current rating, not guaranteed continuous thermal current. Confirm controller and cooling limits.
All candidates use the existing APC 16x16 propeller coefficient curve. This DOES NOT optimize propeller selection; alternatives need verified CT(J), CP(J) data and RPM limits.
Algorithm: compare all manufacturer-data windings at 12S using the 5%-SOC bus voltage screen and detailed discharge simulation. If NONE passes, report this and try higher-voltage reference motor options. Lower gross battery mass wins among feasible candidates in the active architecture.
Reference chemistry values retained; equivalent battery capacity in Ah, not physical integer cells, until an actual cell capacity is supplied.
CAUTION: ESC maximum voltage and current, motor thermal rating, propeller structural RPM limit, pack packaging, and temperature have NOT been validated. Converged weight is conceptual only.
Point-performance analyzer is temporarily bypassed for electric designs because legacy Propulsion.m is not coupled to optimized motor/battery input. Fuel-aircraft performance remains intact.
