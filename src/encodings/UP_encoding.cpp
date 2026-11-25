#include "solver_common.hpp"

// hide includes from user
#include "cadical.hpp" // https://github.com/arminbiere/cadical (submodule in solvers/cadical)
#include "mapf_propagator.hpp"

#include <cstdlib>

using namespace std;

/****************************/
// MARK: constructor
/****************************/

/** Constructor of _MAPFSAT_UP.
*
* At variables only. Vertex, swapping and pebble conflicts (and, with single, the
* at-most-one position per agent and timestep) are enforced by an IPASIR-UP
* propagator connected to CaDiCaL instead of being encoded as clauses.
*
* @param var must be 1 (at).
* @param cost optimized cost function. Possible values 1 = mks, 2 = soc.
* @param moves allowed movement. Possible values 1 = parallel, 2 = pebble.
* @param lazy must be 1 (eager), conflicts are handled by the propagator.
* @param dupli allow of forbid duplicating agents. Possible values 1 = forbid, 2 = allow.
* @param solver must be 3.
* @param sol_name name of the encoding used in log.
*/
_MAPFSAT_UP::_MAPFSAT_UP(int var, int cost, int moves, int lazy, int dupli, int solver, string sol_name)
	: _MAPFSAT_SAT(var, cost, moves, lazy, dupli, 1, sol_name)
{
	solver_to_use = solver;

	assert(variables == 1);
	assert(lazy_const == 1);
	assert(solver_to_use == 3);
}

/****************************/
// MARK: formula
/****************************/

int _MAPFSAT_UP::CreateFormula(int time_left)
{
	int timesteps = inst->GetMksLB(agents) + delta;
	int lit = nr_vars; 	// 1, a new solver is created for every call
	auto start = chrono::high_resolution_clock::now();

	/*****************************************/
	/* variables, propagator and observation */
	/*****************************************/
	lit = CreateAt(lit, timesteps);

	MapfTegSpec spec;
	spec.agents = agents;
	spec.vertices = vertices;
	spec.layers = timesteps;
	spec.num_vars = lit - 1;
	spec.pebble = (movement == 2);
	spec.single = (duplicates == 1);
	// Following paths in cb_decide helps to find plans (makespan is usually
	// satisfiable at the lower bound) but slows down the unsatisfiable calls
	// that dominate sum of costs. MAPF_UP_DECIDE=0/1/2 overrides it.
	const char* decide = getenv("MAPF_UP_DECIDE");
	spec.decide = (decide != NULL) ? atoi(decide) : (cost_function == 1 ? 1 : 0);
	spec.first_var.assign((size_t)agents * vertices, 0);
	spec.first_t.assign((size_t)agents * vertices, 0);
	spec.last_t.assign((size_t)agents * vertices, -1);
	for (int a = 0; a < agents; a++)
	{
		for (int v = 0; v < vertices; v++)
		{
			if (at[a][v].first_variable == 0)
				continue;
			size_t k = (size_t)a * vertices + v;
			spec.first_var[k] = at[a][v].first_variable;
			spec.first_t[k] = at[a][v].first_timestep;
			spec.last_t[k] = at[a][v].last_timestep;
		}
	}
	spec.nbr.assign((size_t)vertices * 5, -1);
	for (int v = 0; v < vertices; v++)
		for (int dir = 0; dir < 5; dir++)
			if (inst->HasNeighbor(v, dir))
				spec.nbr[v * 5 + dir] = inst->GetNeighbor(v, dir);

	// the propagator has to be connected and observe its variables before any clause is added
	MapfPropagator* prop = new MapfPropagator(spec);
	propagator = prop;
	CaDiCaL::Solver* solver = (CaDiCaL::Solver*)SAT_solver;
	solver->connect_external_propagator(prop);
	// Observing a new variable activates it right away, which reallocates the
	// solver's value array every time (quadratic). Freezing first imports all
	// variables lazily, so the first observation activates them in one go.
	for (int x = 1; x < lit; x++)
		solver->freeze(x);
	for (int x = 1; x < lit; x++)
		solver->add_observed_var(x);

	if (TimesUp(start, chrono::high_resolution_clock::now(), time_left))
		return -1;

	/***************************/
	/* start - goal possitions */
	/***************************/
	CreatePossition_Start();
	CreatePossition_Goal();
	if (cost_function == 2)
		CreatePossition_NoneAtGoal();

	/************/
	/* movement */
	/************/
	CreateMove_NextVertex_At();
	const char* backward = getenv("MAPF_UP_BACKWARD");	// experiments: 0 = no backward support clauses
	if (backward == NULL || atoi(backward) != 0)
		CreateMove_PrevVertex_At();

	if (TimesUp(start, chrono::high_resolution_clock::now(), time_left))
		return -1;

	/**************/
	/* limit cost */
	/**************/
	if (cost_function == 2 && delta > 0)
	{
		if (duplicates == 1)
			lit = CreateConst_LimitSoc(lit);
		if (duplicates == 2)
			lit = CreateConst_LimitSoc_AllAt(lit);
	}

	/*******************/
	/* avoid locations */
	/*******************/
	if (use_avoid)
		CreateConst_Avoid();

	if (TimesUp(start, chrono::high_resolution_clock::now(), time_left))
		return -1;

	return lit;
}

// If agent a is at v in timestep t > 0, it was at v or at a neighbour of v in t-1.
void _MAPFSAT_UP::CreateMove_PrevVertex_At()
{
	for (int a = 0; a < agents; a++)
	{
		for (int v = 0; v < vertices; v++)
		{
			if (at[a][v].first_variable == 0)
				continue;

			for (int t = max(1, at[a][v].first_timestep); t <= at[a][v].last_timestep; t++)
			{
				int prev_t = t - 1;
				vector<int> clause;
				for (int dir = 0; dir < 5; dir++)
				{
					if (!inst->HasNeighbor(v, dir))
						continue;
					int u = inst->GetNeighbor(v, dir);
					if (at[a][u].first_variable == 0)
						continue;
					if (at[a][u].first_timestep <= prev_t && at[a][u].last_timestep >= prev_t)
						clause.push_back(at[a][u].first_variable + (prev_t - at[a][u].first_timestep));
				}
				clause.push_back(-(at[a][v].first_variable + (t - at[a][v].first_timestep)));
				AddClause(clause);
				nr_clauses_move++;
			}
		}
	}
}

/****************************/
// MARK: solver
/****************************/

void _MAPFSAT_UP::CreateSolver()
{
	_MAPFSAT_SAT::CreateSolver();
}

void _MAPFSAT_UP::ReleaseSolver()
{
	MapfPropagator* prop = (MapfPropagator*)propagator;
	if (prop != NULL && getenv("MAPF_UP_STATS") != NULL)
	{
		const MapfPropagatorStats& s = prop->stats;
		cerr << "c up stats: true=" << s.notified_true << " false=" << s.notified_false
			<< " kills(vertex,layer,swap,pebble)=" << s.kills_vertex << "," << s.kills_layer << "," << s.kills_swap << "," << s.kills_pebble
			<< " propagated=" << s.propagated << " reasons=" << s.reasons
			<< " model_checks=" << s.model_checks << " rejects=" << s.model_rejects
			<< " backtracks=" << s.backtracks << " decisions=" << s.decisions << endl;
	}
	if (SAT_solver != NULL && prop != NULL)
		((CaDiCaL::Solver*)SAT_solver)->disconnect_external_propagator();
	_MAPFSAT_SAT::ReleaseSolver();
	delete prop;
	propagator = NULL;
}
