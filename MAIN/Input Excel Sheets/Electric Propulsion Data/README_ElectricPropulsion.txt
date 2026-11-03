ELECTRIC PROPULSION + BATTERY INPUTS

Populate ElectricPropulsion_Input_Template.xlsx before sizing electric configurations.

MotorData: one row per electric Config_Row; provide Kv [rpm/V], winding resistance [ohm], no-load current [A], continuous current [A], continuous shaft power [W], and max motor voltage [V]. Pmax_W is interpreted as continuous shaft power by ElectricPropulsion.m.

PropellerData: one row per electric Config_Row; provide diameter [m] and matching semicolon-separated J_data, CT_data, CP_data vectors. Coefficients must come from a consistent propeller test/model data set and include J=0 for static takeoff.

BatteryModelInputs: generic chemistry/reference inputs. Replace illustrative OCV and pack-level energy density values with validated data before using the resulting pack mass as a design value. Vmin_bus_V is the required minimum bus voltage and Vmax_bus_V is the maximum permitted fully charged bus voltage.

Integration order: mission segment inputs -> EvaluateElectricMissionSegment -> ElectricPropulsion operating point -> mission power/duration profile -> BatteryRequirements -> battery mass -> aircraft weight iteration. BatteryRequirements adds avionics and wiring loss, so mission.P_motor_W must contain propulsion electrical input only.

Current mission template limitation: Startup/Taxi, Descent and Landing rows only define a legacy weight fraction, not speed/altitude/duration. The workbook therefore supplies explicit approximate duration and power-fraction defaults for these phases. Replace them with better project-specific estimates. Detailed segment power from the propulsion solver is used for Takeoff, Climb, Cruise and Loiter.

The model is not a substitute for cell-level validation; confirm motor ratings, propeller data, pack resistance/OCV, thermal limits, and all feasibility results before relying on sizing outputs.
