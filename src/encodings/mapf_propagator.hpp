#ifndef _mapf_propagator_h_INCLUDED
#define _mapf_propagator_h_INCLUDED

// IPASIR-UP propagator for the collision constraints of MAPF.
//
// The At(a,v,t) variables of all agents are observed. Instead of adding the
// O(agents^2 * vertices * timesteps) conflict clauses up front, the
// propagator enforces them during search and explains every propagation
// with the corresponding (binary or swap) conflict clause, which CaDiCaL
// only requests when the propagation takes part in a conflict.
//
// Not part of the public MAPF.hpp header (it needs cadical.hpp).

#include <cstdint>
#include <vector>

#include "cadical.hpp"

// Static description of the time-expanded graphs.
// Node (a,v,t) exists iff first_var[a*V+v] != 0 and first_t <= t <= last_t,
// its variable is first_var + (t - first_t). Variables are 1..num_vars.
struct MapfTegSpec
{
	int agents = 0;
	int vertices = 0;
	int layers = 0;					// timesteps 0..layers-1
	int num_vars = 0;				// At variables are exactly 1..num_vars
	bool pebble = false;			// pebble motion, otherwise parallel motion
	bool single = false;			// at most one node per (agent, timestep)
	int decide = 0;					// cb_decide: 0 = off, 1 = agents in order, 2 = shortest prefix first
	std::vector<int> first_var;		// [a*V+v], 0 = vertex not used by agent a
	std::vector<int> first_t;		// [a*V+v]
	std::vector<int> last_t;		// [a*V+v]
	std::vector<int> nbr;			// [v*5+d], d = 0 wait, 1..4 N S W E, -1 = none
};

struct MapfPropagatorStats
{
	long long notified_true = 0;
	long long notified_false = 0;
	long long kills_vertex = 0;		// other agent at the same vertex and time
	long long kills_layer = 0;		// same agent elsewhere at the same time (single)
	long long kills_swap = 0;
	long long kills_pebble = 0;
	long long propagated = 0;		// literals returned by cb_propagate
	long long reasons = 0;			// reason clauses requested by the solver
	long long model_checks = 0;
	long long model_rejects = 0;
	long long backtracks = 0;
	long long decisions = 0;
};

class MapfPropagator : public CaDiCaL::ExternalPropagator
{
public:
	explicit MapfPropagator(const MapfTegSpec& spec);

	void notify_assignment(const std::vector<int>& lits) override;
	void notify_new_decision_level() override;
	void notify_backtrack(size_t new_level) override;
	bool cb_check_found_model(const std::vector<int>& model) override;
	int cb_propagate() override;
	int cb_decide() override;
	int cb_add_reason_clause_lit(int propagated_lit) override;
	bool cb_has_external_clause(bool& is_forgettable) override;
	int cb_add_external_clause_lit() override;

	MapfPropagatorStats stats;

private:
	enum UndoType { UNDO_VAL = 0, UNDO_DEAD = 1, UNDO_OCC = 2, UNDO_LTRUE = 3, UNDO_FRONTIER = 4 };
	struct Undo { int type; int idx; int old; };
	struct Mark { size_t undo; size_t outq; size_t pool; };

	MapfTegSpec S;
	int A, V, L, N;

	// static node information, indexed by variable
	std::vector<int> node_agent, node_vertex, node_time;
	// agents that have at least one node at vertex v (CSR)
	std::vector<int> agents_at_start, agents_at_list;
	// nodes of layer (a,t) (CSR), only built in single mode
	std::vector<int> layer_start, layer_list;

	// dynamic state
	std::vector<int8_t> val;		// mirror of notified assignments
	std::vector<uint8_t> dead;		// a propagation of -var is queued or done
	std::vector<int> occ;			// [v*L+t] = 1 + agent with a true node there, 0 = none
	std::vector<int> ltrue;			// [a*L+t] = a true node of agent a at t, 0 = none
	std::vector<int> frontier;		// [a] = first timestep without a true node of agent a
	std::vector<int> agent_last;	// [a] = last timestep of agent a (its goal)
	std::vector<Undo> undo;
	std::vector<Mark> marks;		// one per decision level above 0
	std::vector<int> outq;			// literals to propagate, delivered in order
	size_t head = 0;
	std::vector<int> pool;			// reason clauses: [len, propagated lit, other lits...]
	std::vector<int> reason_off;	// [var] offset into pool (valid while dead[var])
	int reason_cur = -1, reason_end = -1;
	std::vector<int> ext;			// external clause pending after a rejected model
	size_t ext_pos = 0;
	bool ext_pending = false;

	inline int var_at(int a, int v, int t) const
	{
		if (t < 0 || t >= L)
			return 0;
		int k = a * V + v;
		int f = S.first_var[k];
		if (!f || t < S.first_t[k] || t > S.last_t[k])
			return 0;
		return f + (t - S.first_t[k]);
	}
	inline int occupant(int v, int t) const	// agent with a true node at (v,t), or -1
	{
		if (v < 0 || t < 0 || t >= L)
			return -1;
		return occ[(size_t)v * L + t] - 1;
	}

	void on_true(int n);
	void kill(int m, int o1, int o2 = 0, int o3 = 0);
	void swap_move(int a, int from, int to, int t, int lit_from, int lit_to);
	bool find_violation(const std::vector<int>& model);
};

#endif
