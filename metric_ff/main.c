/*********************************************************************
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 * 
 *********************************************************************/

/*********************************************************************
 * File: main.c
 * Description: The main routine for the Metric-FastForward Planner.
 *              Modified July 2011 to allow more command-line search 
 *              configurations, including improved cost-minimization
 *
 * Author: original version Joerg Hoffmann 2001/2002
 *         modified version Joerg Hoffmann 2012
 * 
 *********************************************************************/ 




#if defined(PYTHON)
#include <python2.7/Python.h>
PyObject* FFError;
#endif



#include "ff.h"

#include "memory.h"
#include "output.h"

#include "parse.h"

#include "expressions.h"

#include "inst_pre.h"
#include "inst_easy.h"
#include "inst_hard.h"
#include "inst_final.h"

#include "relax.h"
#include "search.h"






/*
 *  ----------------------------- GLOBAL VARIABLES ----------------------------
 */


/*******************
 * GENERAL HELPERS *
 *******************/


/* used to time the different stages of the planner
 */
float gtempl_time = 0, greach_time = 0, grelev_time = 0, gconn_time = 0;
float gLNF_time = 0, gsearch_time = 0;


/* the command line inputs
 */
struct _command_line gcmd_line;

/* number of states that got heuristically evaluated
 */
int gevaluated_states = 0;

/* maximal depth of breadth first search
 */
int gmax_search_depth = 0;

/* limit for depth search in hill climbing*/
int glimit_search_depth = -1;


/***********
 * PARSING *
 ***********/







/* used for pddl parsing, flex only allows global variables
 */
int gbracket_count;
char *gproblem_name;

/* The current input line number
 */
int lineno = 1;

/* The current input filename
 */
char *gact_filename = NULL;

/* The pddl domain name
 */
Token gdomain_name = NULL;

/* loaded, uninstantiated operators
 */
PlOperator *gloaded_ops = NULL;

/* stores initials as fact_list 
 */
PlNode *gorig_initial_facts = NULL;

/* not yet preprocessed goal facts
 */
PlNode *gorig_goal_facts = NULL;

/* axioms as in UCPOP before being changed to ops
 */
PlOperator *gloaded_axioms = NULL;

/* the types, as defined in the domain file
 */
TypedList *gparse_types = NULL;

/* the constants, as defined in domain file
 */
TypedList *gparse_constants = NULL;

/* the predicates and their arg types, as defined in the domain file
 */
TypedListList *gparse_predicates = NULL;

/* the functions and their arg types, as defined in the domain file
 */
TypedListList *gparse_functions = NULL;

/* the objects, declared in the problem file
 */
TypedList *gparse_objects = NULL;

/* the metric
 */
Token gparse_optimization = NULL;
ParseExpNode *gparse_metric = NULL;


/* connection to instantiation ( except ops, goal, initial )
 */

/* all typed objects 
 */
FactList *gorig_constant_list = NULL;

/* the predicates and their types
 */
FactList *gpredicates_and_types = NULL;

/* the functions and their types
 */
FactList *gfunctions_and_types = NULL;












/*****************
 * INSTANTIATING *
 *****************/









/* global arrays of constant names,
 *               type names (with their constants),
 *               predicate names,
 *               predicate aritys,
 *               defined types of predicate args
 */
Token gconstants[MAX_CONSTANTS];
int gnum_constants = 0;
Token gtype_names[MAX_TYPES];
int gtype_consts[MAX_TYPES][MAX_TYPE];
Bool gis_member[MAX_CONSTANTS][MAX_TYPES];
int gmember_nr[MAX_CONSTANTS][MAX_TYPES];/* nr of object within a type */
int gtype_size[MAX_TYPES];
int gnum_types = 0;
Token gpredicates[MAX_PREDICATES];
int garity[MAX_PREDICATES];
Bool gaxiom_added[MAX_PREDICATES];
int gpredicates_args_type[MAX_PREDICATES][MAX_ARITY];
int gnum_predicates = 0;
Token gfunctions[MAX_FUNCTIONS];
int gf_arity[MAX_FUNCTIONS];
int gfunctions_args_type[MAX_FUNCTIONS][MAX_ARITY];
int gnum_functions = 0;





/* the domain in integer (Fact) representation
 */
Operator_pointer goperators[MAX_OPERATORS];
int gnum_operators = 0;
Fact *gfull_initial = NULL;
int gnum_full_initial = 0;
FluentValue *gfull_fluents_initial = NULL;
int gnum_full_fluents_initial = 0;
WffNode *ggoal = NULL;

ExpNode *gmetric = NULL;



/* stores inertia - information: is any occurence of the predicate
 * added / deleted in the uninstantiated ops ?
 */
Bool gis_added[MAX_PREDICATES];
Bool gis_deleted[MAX_PREDICATES];


/* for functions we *might* want to say, symmetrically, whether it is
 * increased resp. decreased at all.
 *
 * that is, however, somewhat involved because the right hand
 * sides can be arbirtray expressions, so we have no guarantee
 * that increasing really does adds to a functions value...
 *
 * thus (for the time being), we settle for "is the function changed at all?"
 */
Bool gis_changed[MAX_FUNCTIONS];



/* splitted initial state:
 * initial non static facts,
 * initial static facts, divided into predicates
 * (will be two dimensional array, allocated directly before need)
 */
Facts *ginitial = NULL;
int gnum_initial = 0;
Fact **ginitial_predicate = NULL;
int *gnum_initial_predicate = NULL;

/* same thing for functions
 */
FluentValues *gf_initial = NULL;
int gnum_f_initial = 0;
FluentValue **ginitial_function = NULL;
int *gnum_initial_function = NULL;



/* the type numbers corresponding to any unary inertia
 */
int gtype_to_predicate[MAX_PREDICATES];
int gpredicate_to_type[MAX_TYPES];

/* (ordered) numbers of types that new type is intersection of
 */
TypeArray gintersected_types[MAX_TYPES];
int gnum_intersected_types[MAX_TYPES];



/* splitted domain: hard n easy ops
 */
Operator_pointer *ghard_operators = NULL;
int gnum_hard_operators = 0;
NormOperator_pointer *geasy_operators = NULL;
int gnum_easy_operators = 0;



/* so called Templates for easy ops: possible inertia constrained
 * instantiation constants
 */
EasyTemplate *geasy_templates = NULL;
int gnum_easy_templates = 0;



/* first step for hard ops: create mixed operators, with conjunctive
 * precondition and arbitrary effects
 */
MixedOperator *ghard_mixed_operators = NULL;
int gnum_hard_mixed_operators = 0;



/* hard ''templates'' : pseudo actions
 */
PseudoAction_pointer *ghard_templates = NULL;
int gnum_hard_templates = 0;



/* store the final "relevant facts"
 */
Fact grelevant_facts[MAX_RELEVANT_FACTS];
int gnum_relevant_facts = 0;
int gnum_pp_facts = 0;
/* store the "relevant fluents"
 */
Fluent grelevant_fluents[MAX_RELEVANT_FLUENTS];
int gnum_relevant_fluents = 0;
Token grelevant_fluents_name[MAX_RELEVANT_FLUENTS];
/* this is NULL for normal, and the LNF for
 * artificial fluents.
 */
LnfExpNode_pointer grelevant_fluents_lnf[MAX_RELEVANT_FLUENTS];



/* the final actions and problem representation
 */
Action *gactions = NULL;
int gnum_actions = 0;
State ginitial_state;
int *glogic_goal = NULL;
int gnum_logic_goal = 0;
Comparator *gnumeric_goal_comp = NULL;
ExpNode_pointer *gnumeric_goal_lh = NULL, *gnumeric_goal_rh = NULL;
int gnum_numeric_goal = 0;



/* to avoid memory leaks; too complicated to identify
 * the exact state of the action to throw away (during construction),
 * memory gain not worth the implementation effort.
 */
Action *gtrash_actions = NULL;



/* additional lnf step between finalized inst and
 * conn graph
 */
Comparator *glnf_goal_comp = NULL;
LnfExpNode_pointer *glnf_goal_lh = NULL;
float *glnf_goal_rh = NULL;
int gnum_lnf_goal = 0;

LnfExpNode glnf_metric;
Bool goptimization_established = FALSE;







/**********************
 * CONNECTIVITY GRAPH *
 **********************/







/* one ops (actions) array ...
 */
OpConn *gop_conn = NULL;
int gnum_op_conn = 0;



/* one effects array ...
 */
EfConn *gef_conn = NULL;
int gnum_ef_conn = 0;



/* one facts array.
 */
FtConn *gft_conn = NULL;
int gnum_ft_conn = 0;



/* and: one fluents array.
 */
FlConn *gfl_conn = NULL;
int gnum_fl_conn = 0;
int gnum_real_fl_conn = 0;/* number of non-artificial ones */



/* final goal is also transformed one more step.
 */
int *gflogic_goal = NULL;
int gnum_flogic_goal = 0;
Comparator *gfnumeric_goal_comp = NULL;
int *gfnumeric_goal_fl = NULL;
float *gfnumeric_goal_c = NULL;
int gnum_fnumeric_goal = 0;

/* direct access (by relevant fluents)
 */
Comparator *gfnumeric_goal_direct_comp = NULL;
float *gfnumeric_goal_direct_c = NULL;






/*******************
 * SEARCHING NEEDS *
 *******************/







/* applicable actions
 */
int *gA = NULL;/* non-axioms */
int gnum_A;
int *gA_axioms = NULL; /* axioms */
int gnum_A_axioms;



/* communication from extract 1.P. to search engine:
 * 1P action choice
 */
int *gH = NULL;
int gnum_H;
/* added cost of relaxed plan
 */
float gh_cost;
/* hmax value
 */
float ghmax;



/* to store plan
 */
int gplan_ops[MAX_PLAN_LENGTH];
int gnum_plan_ops = 0;



/* stores the states that the current plan goes through
 * ( for knowing where new agenda entry starts from )
 */
State gplan_states[MAX_PLAN_LENGTH + 1];




/* dirty: multiplic. of total-time in final metric LNF
 */
float gtt;




/* the mneed structures
 */
Bool **gassign_influence = NULL;
Bool **gTassign_influence = NULL;



/* the real var input to the mneed computation.
 */
Bool *gmneed_start_D = NULL;
float *gmneed_start_V = NULL;



/* does this contain conditional effects?
 * (if it does then the state hashing has to be made more
 *  cautiously)
 */
Bool gconditional_effects;



/* easier to question: are we optimizing or no?
 */
Bool gcost_minimizing;



/* stores current A* weight: this is initially given by user,
 * but changes during anytime search.
 */
float gw;
/* this is the minimum weight, ie we'll stop once the weight update
 * does/would yield a value <= this.
 * if no such minim weight is given, this will be -1
 */
float gmin_w = -1;



/* this one says whether or not we are actually using
 * cost-minimizing rplans.
 * this will be the case by default if we're running cost-
 * minimizing searches. it can be switched off by a flag;
 * it is automatically switched off in case there are
 * numeric preconditions/goals: for this case,
 * cost-minimizing rplans are not implemented (a numeric prec
 * may cause an action to come in "later" on in the RPG although
 * its logical pres are easy. in that case, any new effects will
 * have a smaller RPGcost value than facts we already have waiting.
 * in other words, the "Dijsktra" nature breaks.
 *
 * ... I suppose there may be a generic solution to this that
 * can handle numeric precs/goals. Doesn't seem important enough
 * to bother.
 */
Bool gcost_rplans;


/* first_call stuff */
Bool inst_pre_dnf_first_call = TRUE;
Bool relax_collect_1P_and_A_info = TRUE;
Bool relax_collect_A_axioms_info = TRUE;
Bool relax_extract_1P = TRUE;
Bool relax_collect_A_info = TRUE;
Bool relax_initialize_fixpoint = TRUE;
Bool relax_initialize_goals = TRUE;
Bool relax_collect_H_info = TRUE;
Bool search_search_for_better_state = TRUE;
Bool search_expand_first_node = TRUE;
Bool search_add_to_Astar_epsilon_space = TRUE;
Bool search_extract_plan = TRUE;
Bool get_special_plan_first_call = TRUE;
Bool search_result_to_dest = TRUE;
Bool search_do_axiom_update = TRUE;

/*
 *  ----------------------------- HEADERS FOR PARSING ----------------------------
 * ( fns defined in the scan-* files )
 */







void load_ops_file( char *filename );
void load_fct_file( char *filename );
#if defined(PYTHON)
static PyObject* doActualPlanning(Bool pythonMode);
static PyObject* get_special_plan( BfsNode *last );
static PyObject* get_regular_plan( void );
#else
static void doActualPlanning(Bool pythonMode);
#endif
/*
 *  ----------------------------- MAIN ROUTINE ----------------------------
 */



struct tms lstart, lend;
static struct tms start, end;
static Bool prev_gcost_rplans;
static Bool found_plan;


static void cleanup(){
	int x = 0;
	clear_State(&ginitial_state);
	gtempl_time = greach_time = grelev_time = gconn_time = 0;
	gLNF_time = gsearch_time = 0;
	gevaluated_states = 0;
	gmax_search_depth = 0;
	gbracket_count = 0;
	lineno = 1;
	if(gdomain_name != NULL)
		free(gdomain_name);
	gdomain_name = NULL;
	if(gloaded_ops != NULL)
		free_PlOperator(gloaded_ops);
	gloaded_ops = NULL;
	if(gorig_initial_facts != NULL)
		free_PlNode(gorig_initial_facts);
	gorig_initial_facts = NULL;
	if(gorig_goal_facts != NULL)
		free_PlNode(gorig_goal_facts);
	gorig_goal_facts = NULL;
	if(gloaded_axioms != NULL)
		free_PlOperator(gloaded_axioms);
	gloaded_axioms = NULL;
	if(gparse_types != NULL)
		free_TypedList(gparse_types);
	gparse_types = NULL;
	if(gparse_constants != NULL)
		free_TypedList(gparse_constants);
	gparse_constants = NULL;
	if(gparse_predicates != NULL)
		free_TypedListList(gparse_predicates);
	gparse_predicates = NULL;
	if(gparse_functions != NULL)
		free_TypedListList(gparse_functions);
	gparse_functions = NULL;
	if(gparse_objects != NULL)
		free_TypedList(gparse_objects);
	gparse_objects = NULL;
	if(gparse_optimization != NULL)
		free(gparse_optimization);
	gparse_optimization = NULL;
	if(gparse_metric != NULL)
		free_ParseExpNode(gparse_metric);
	gparse_metric = NULL;
	gorig_constant_list = NULL; /* has already been freed by using function collect_all_strings() in inst_pre.c */
	gpredicates_and_types = NULL; /* has already been freed by using function collect_all_strings() in inst_pre.c */
	gfunctions_and_types = NULL; /* has already been freed by using function collect_all_strings() in inst_pre.c */
	for(x = 0; x < gnum_constants; x++)
		if(gconstants[x] != NULL)
			free(gconstants[x]);
	gnum_constants = 0;
	for(x = 1; x < gnum_types; x++) /* the first is always a pointer to a constant ("ARTFICIAL-ALL-OBJECTS") that which we cannot free */
		if(gtype_names[x] != NULL)
			free(gtype_names[x]);
	gnum_types = 0;
	for(x = 1; x < gnum_predicates; x++) /* the first is always a pointer to a constant ("=") that which we cannot free */
		if(gpredicates[x] != NULL)
			free(gpredicates[x]);
	for(x = 1; x < gnum_functions; x++) /* the first is always a pointer to a constant ("TOTAL-TIME") that which we cannot free */
		if(gfunctions[x] != NULL)
			free(gfunctions[x]);
	for(x = 0; x < gnum_operators; x++)
		if(goperators[x] != NULL)
			free_Operator(goperators[x]);
	gnum_operators = 0;
	if(gfull_initial != NULL)
		free(gfull_initial);
	gfull_initial = NULL;
	gnum_full_initial = 0;
	if(gfull_fluents_initial != NULL)
		free(gfull_fluents_initial);
	gfull_fluents_initial = NULL;
	gnum_full_fluents_initial = 0;
	if(ggoal != NULL)
		free_WffNode(ggoal);
	ggoal = NULL;
	if(gmetric != NULL)
		free_ExpNode(gmetric);
	gmetric = NULL;
	if(ginitial != NULL)
		free_Facts(ginitial);
	ginitial = NULL;
	gnum_initial = 0;
	if(ginitial_predicate != NULL){
		for(x = 0; x < gnum_predicates; x++)
			if(ginitial_predicate[x] != NULL){
				free(ginitial_predicate[x]);
				ginitial_predicate[x] = NULL;
			}
		free(ginitial_predicate);
	}
	ginitial_predicate = NULL;
	if(gnum_initial_predicate != NULL)
		free(gnum_initial_predicate);
	gnum_initial_predicate = NULL;
	gnum_predicates = 0;
	if(gf_initial != NULL)
		free_FluentValues(gf_initial);
	gf_initial = NULL;
	gnum_f_initial = 0;
	if(ginitial_function != NULL){
		for(x = 0; x < gnum_functions; x++)
			if(ginitial_function[x] != NULL){
				free(ginitial_function[x]);
				ginitial_function[x] = NULL;
			}
		free(ginitial_function);
	}
	ginitial_function = NULL;
	if(gnum_initial_function != NULL)
		free(gnum_initial_function);
	gnum_initial_function = NULL;
	gnum_functions = 0;
	if(ghard_operators != NULL) /* ghard_operators only stores pointers to operators owned by Operators so there is no need to free contents here. */
		free(ghard_operators);
	ghard_operators = NULL;
	gnum_hard_operators = 0;
	if(geasy_operators != NULL){
		for(x = 0; x < gnum_easy_operators; x++)
			if(geasy_operators[x] != NULL){
				free_NormOperator(geasy_operators[x]);
				geasy_operators[x] = NULL;
			}
	}
	geasy_operators = NULL;
	gnum_easy_operators = 0;
	if(geasy_templates != NULL)
		free_single_EasyTemplate(geasy_templates); /* Well, is this enough cleanup? */
	geasy_templates = NULL;
	gnum_easy_templates = 0;
	if(ghard_mixed_operators != NULL)
		free_MixedOperator(ghard_mixed_operators);
	ghard_mixed_operators = NULL;
	gnum_hard_mixed_operators = 0;
	if(ghard_templates != NULL){
		for(x = 0; x < gnum_hard_templates; x++)
			if(ghard_templates[x] != NULL){
				free(ghard_templates[x]);
				ghard_templates[x] = NULL;
			}
		free(ghard_templates);
	}
	ghard_templates = NULL;
	gnum_hard_templates = 0;
	gnum_relevant_facts = 0;
	gnum_pp_facts = 0;
	for(x = 0; x < gnum_relevant_fluents; x++){
		if(grelevant_fluents_name[x] != NULL)
			free(grelevant_fluents_name[x]);
		if(grelevant_fluents_lnf[x])
			free(grelevant_fluents_lnf[x]);
	}
	gnum_relevant_fluents = 0;
	if(gactions != NULL)
		free(gactions);
	gactions = NULL;
	gnum_actions = 0;
	if(glogic_goal != NULL)
		free(glogic_goal);
	glogic_goal = NULL;
	gnum_logic_goal = 0;
	if(gnumeric_goal_comp != NULL)
		free(gnumeric_goal_comp);
	gnumeric_goal_comp = NULL;
	if(gnumeric_goal_lh != NULL){
		for(x = 0; x < gnum_numeric_goal; x++)
			if(gnumeric_goal_lh[x]){
				free_ExpNode(gnumeric_goal_lh[x]);
				gnumeric_goal_lh[x] = NULL;
			}
		free(gnumeric_goal_lh);
	}
	gnumeric_goal_lh = NULL;
	if(gnumeric_goal_rh != NULL){
		for(x = 0; x < gnum_numeric_goal; x++)
			if(gnumeric_goal_rh[x]){
				free_ExpNode(gnumeric_goal_rh[x]);
				gnumeric_goal_rh[x] = NULL;
			}
		free(gnumeric_goal_rh);
	}
	gnumeric_goal_rh = NULL;
	gnum_numeric_goal = 0;
	gtrash_actions = NULL; /* TODO: This is unused anyway (at least, I do not get what it is actually for ...) */
	if(glnf_goal_comp != NULL)
		free(glnf_goal_comp);
	glnf_goal_comp = NULL;
	if(glnf_goal_lh)
		free(glnf_goal_lh);
	glnf_goal_lh = NULL;
	if(glnf_goal_rh != NULL)
		free(glnf_goal_rh);
	glnf_goal_rh = NULL;
	gnum_lnf_goal = 0;
	goptimization_established = FALSE;
	/* TODO: the following 4 connections structures are not that easy to free: They contain dynamically allocated members */
	if(gop_conn != NULL)
		free(gop_conn);
	gop_conn = NULL;
	gnum_op_conn = 0;
	if(gef_conn != NULL)
		free(gef_conn);
	gef_conn = NULL;
	gnum_ef_conn = 0;
	if(gft_conn != NULL)
		free(gft_conn);
	gft_conn = NULL;
	gnum_ft_conn = 0;
	if(gfl_conn)
		free(gfl_conn);
	gfl_conn = NULL;
	gnum_fl_conn = 0;
	/* Tricky connections ending here */
	if(gflogic_goal != NULL)
		free(gflogic_goal);
	gflogic_goal = NULL;
	gnum_flogic_goal = 0;
	if(gfnumeric_goal_comp != NULL)
		free(gfnumeric_goal_comp);
	gfnumeric_goal_comp = NULL;
	if(gfnumeric_goal_fl)
		free(gfnumeric_goal_fl);
	gfnumeric_goal_fl = NULL;
	if(gfnumeric_goal_c)
		free(gfnumeric_goal_c);
	gfnumeric_goal_c = NULL;
	gnum_fnumeric_goal = 0;
	if(gfnumeric_goal_direct_comp != NULL)
		free(gfnumeric_goal_direct_comp);
	gfnumeric_goal_direct_comp = NULL;
	if(gfnumeric_goal_direct_c != NULL)
		free(gfnumeric_goal_direct_c);
	gfnumeric_goal_direct_c = NULL;
	if(gA != NULL)
		free(gA);
	gA= NULL;
	gnum_A = 0;
	if(gA_axioms != NULL)
		free(gA_axioms);
	gA_axioms = NULL;
	gnum_A_axioms = 0;
	if(gH != NULL)
		free(gH);
	gH = NULL;
	gnum_H = 0;
	gh_cost = 0.0f;
	ghmax = 0.0f;
	gnum_plan_ops = 0;
	gtt = 0.0f;
	if(gassign_influence != NULL){
		for(x = 0; x < gnum_real_fl_conn; x++)
			if(gassign_influence[x] != NULL){
				free(gassign_influence[x]);
				gassign_influence[x] = NULL;
			}
		free(gassign_influence);
	}
	gassign_influence = NULL;
	if(gTassign_influence != NULL){
		for(x = 0; x < gnum_real_fl_conn; x++)
			if(gTassign_influence[x] != NULL){
				free(gTassign_influence[x]);
				gTassign_influence[x] = NULL;
			}
		free(gTassign_influence);
	}
	gTassign_influence = NULL;
	gnum_real_fl_conn = 0;
	if(gmneed_start_D != NULL)
		free(gmneed_start_D);
	gmneed_start_D = NULL;
	if(gmneed_start_V != NULL)
		free(gmneed_start_V);
	gmneed_start_V = NULL;
	gmin_w = -1;
	inst_pre_dnf_first_call = TRUE;
	relax_collect_1P_and_A_info = TRUE;
	relax_collect_A_axioms_info = TRUE;
	relax_extract_1P = TRUE;
	relax_collect_A_info = TRUE;
	relax_initialize_fixpoint = TRUE;
	relax_initialize_goals = TRUE;
	relax_collect_H_info = TRUE;
	search_search_for_better_state = TRUE;
	search_expand_first_node = TRUE;
	search_add_to_Astar_epsilon_space = TRUE;
	search_extract_plan = TRUE;
	search_result_to_dest = TRUE;
	search_do_axiom_update = TRUE;
	get_special_plan_first_call = TRUE;
}


int main( int argc, char *argv[] ){

    /* resulting name for ops file
     */
    char ops_file[MAX_LENGTH] = "";
    /* same for fct file
     */
    char fct_file[MAX_LENGTH] = "";


    int i;
    float cost;

    times ( &lstart );

    /* command line treatment
     */
    gcmd_line.display_info = 1;
    gcmd_line.debug = 0;

    /* search settings
     */
    gcmd_line.search_config = 5;
    gcmd_line.cost_rplans = TRUE;
    gcmd_line.w = 5;
    gcmd_line.cost_bound = -1;

    memset(gcmd_line.ops_file_name, 0, MAX_LENGTH);
    memset(gcmd_line.fct_file_name, 0, MAX_LENGTH);
    memset(gcmd_line.path, 0, MAX_LENGTH);

    if ( argc == 1 || ( argc == 2 && *++argv[0] == '?' ) ) {
        ff_usage();
        exit( 1 );
    }
    if ( !process_command_line( argc, argv ) ) {
        ff_usage();
        exit( 1 );
    }


    /* make file names
     */

    /* one input name missing
     */
    if ( !gcmd_line.ops_file_name ||
            !gcmd_line.fct_file_name ) {
        fprintf(stdout, "\nff: two input files needed\n\n");
        ff_usage();
        exit( 1 );
    }
    /* add path info, complete file names will be stored in
     * ops_file and fct_file
     */
    sprintf(ops_file, "%s%s", gcmd_line.path, gcmd_line.ops_file_name);
    sprintf(fct_file, "%s%s", gcmd_line.path, gcmd_line.fct_file_name);


    /* parse the input files
     */

    /* start parse & instantiation timing
     */
    times( &start );
    /* domain file (ops)
     */
    if ( gcmd_line.display_info >= 1 ) {
        printf("\nff: parsing domain file");
    }
    /* it is important for the pddl language to define the domain before
     * reading the problem
     */
    load_ops_file( ops_file );
    /* problem file (facts)
     */
    if ( gcmd_line.display_info >= 1 ) {
        printf(" ... done.\nff: parsing problem file");
    }
    load_fct_file( fct_file );
    if ( gcmd_line.display_info >= 1 ) {
        printf(" ... done.\n\n");
    }

    doActualPlanning(FALSE);

    output_planner_info();

    cleanup();

    exit( 0 );
}


/*
 *  ----------------------------- HELPING FUNCTIONS ----------------------------
 */

#if defined(PYTHON)
static PyObject* doActualPlanning(Bool pythonMode){
#else
static void doActualPlanning(Bool pythonMode){
#endif

    /* This is needed to get all types.
     */
    build_orig_constant_list();

    /* last step of parsing: see if it's an ADL domain!
     */
    if ( !make_adl_domain() ) {
        printf("\nff: this is not an ADL problem!");
        printf("\n    can't be handled by this version.\n\n");
        exit( 1 );
    }

    /* now instantiate operators;
     */

    /**************************
     * first do PREPROCESSING *
     **************************/

    /* start by collecting all strings and thereby encoding
     * the domain in integers.
     */
    encode_domain_in_integers();

    /* inertia preprocessing, first step:
     *   - collect inertia information
     *   - split initial state into
     *        - arrays for individual predicates
     *        - arrays for all static relations
     *        - array containing non - static relations
     */
    do_inertia_preprocessing_step_1();

    /* normalize all PL1 formulae in domain description:
     * (goal, preconds and effect conditions)
     *   - simplify formula
     *   - expand quantifiers
     *   - NOTs down
     */
    normalize_all_wffs();

    /* translate negative preconds: introduce symmetric new predicate
     * NOT-p(..) (e.g., not-in(?ob) in briefcaseworld)
     */
    translate_negative_preconds();

    /* split domain in easy (disjunction of conjunctive preconds)
     * and hard (non DNF preconds) part, to apply
     * different instantiation algorithms
     */
    split_domain();

    /***********************************************
     * PREPROCESSING FINISHED                      *
     *                                             *
     * NOW MULTIPLY PARAMETERS IN EFFECTIVE MANNER *
     ***********************************************/

    build_easy_action_templates();
    build_hard_action_templates();

    times( &end );
    TIME( gtempl_time );

    times( &start );

    /* perform reachability analysis in terms of relaxed
     * fixpoint
     */
    perform_reachability_analysis();

    times( &end );
    TIME( greach_time );

    times( &start );

    /* collect the relevant facts and build final domain
     * and problem representations.
     */
    collect_relevant_facts_and_fluents();

    times( &end );
    TIME( grelev_time );


    /* now transform problem to additive normal form,
     * if possible
     */
    times( &start );
    if ( !transform_to_LNF() ) {
        printf("\n\nThis is not a linear task!\n\n");
        exit( 1 );
    }
    times( &end );
    TIME( gLNF_time );

    times( &start );

    /* now build globally accessable connectivity graph
     */
    build_connectivity_graph();

    /* now check for acyclic := effects (in expressions.c)
     */
    check_assigncycles();
    /* set the relevanc info (in expressions.c)
     */
    determine_fl_relevance();

    times( &end );
    TIME( gconn_time );

    /***********************************************************
     * we are finally through with preprocessing and can worry *
     * bout finding a plan instead.                            *
     ***********************************************************/

    if ( gcmd_line.display_info ) {
        printf("\n\nff: search configuration is ");
        switch ( gcmd_line.search_config ) {
        case 0:
            printf("Enforced Hill-Climbing, if that fails then best-first search.\nMetric is plan length.");
            printf("\nNO COST MINIMIZATION");
            if ( !gcost_rplans ) {
                printf(" (and no cost-minimizing relaxed plans).");
            } else {
                printf("\nDEBUG ME: cost min rplans in non-cost minimizing search config?\n\n");
                exit( 1 );
            }
            break;
        case 1:
            printf("best-first search.\nMetric is plan length.");
            printf("\nNO COST MINIMIZATION");
            if ( !gcost_rplans ) {
                printf(" (and no cost-minimizing relaxed plans).");
            } else {
                printf("\nDEBUG ME: cost min rplans in non-cost minimizing search config?\n\n");
                exit( 1 );
            }
            break;
        case 2:
            printf("best-first search with helpful actions pruning.\nMetric is plan length.");
            printf("\nNO COST MINIMIZATION.");
            if ( !gcost_rplans ) {
                printf(" (and no cost-minimizing relaxed plans).");
            } else {
                printf("\nDEBUG ME: cost min rplans in non-cost minimizing search config?\n\n");
                exit( 1 );
            }
            break;
        case 3:
            printf("weighted A* with weight %d.", gcmd_line.w);
            if ( goptimization_established ) {
                printf("\nMetric is ");
                print_LnfExpNode( &glnf_metric );
            } else {
                printf(" plan length");
            }
            printf("\nCOST MINIMIZATION DONE");
            if ( !gcost_rplans ) {
                printf(" (WITHOUT cost-minimizing relaxed plans).");
            } else {
                printf(" (WITH cost-minimizing relaxed plans).");
            }
            break;
        case 4:
            printf("A*epsilon with weight %d.", gcmd_line.w);
            if ( goptimization_established ) {
                printf("\nMetric is ");
                print_LnfExpNode( &glnf_metric );
            } else {
                printf("\nError! Optimization criterion not established.\nA*epsilon not defined.\n\n");
                exit( 1 );
            }
            printf("\nCOST MINIMIZATION DONE");
            if ( !gcost_rplans ) {
                printf(" (WITHOUT cost-minimizing relaxed plans).");
            } else {
                printf(" (WITH cost-minimizing relaxed plans).");
            }
            break;
        case 5:
            printf("Enforced Hill-Climbing, then A*epsilon with weight %d.", gcmd_line.w);
            if ( goptimization_established ) {
                printf("\nMetric is ");
                print_LnfExpNode( &glnf_metric );
            } else {
                printf("\nError! Optimization criterion not established.\nA*epsilon not defined.\n\n");
                exit( 1 );
            }
            printf("\nCOST MINIMIZATION DONE");
            if ( !gcost_rplans ) {
                printf(" (WITHOUT cost-minimizing relaxed plans).");
            } else {
                printf(" (WITH cost-minimizing relaxed plans).");
            }
            break;
        default:
            printf("\n\nUnknown search configuration: %d\n\n", gcmd_line.search_config );
            exit( 1 );
        }
    } else {
        if ( gcmd_line.search_config == 4 && !goptimization_established ) {
            printf("\nError! Optimization criterion not established.\nA*epsilon not defined.\n\n");
            exit( 1 );
        }
    }


    times( &start );



    /* need to evaluate derived predicates in initial state!
     */
    do_axiom_update( &ginitial_state );


    if ( !gcost_rplans ) {
        gcmd_line.cost_bound = -1;
    }
    BfsNode* firstStep = NULL;

    switch ( gcmd_line.search_config ) {
    case 0:
        found_plan = do_enforced_hill_climbing();
        if ( found_plan ) {
            if ( gcmd_line.display_info ) {
                print_plan();
            }
#if defined(PYTHON)
            if(pythonMode)
                return get_regular_plan();
#endif
        } else {
            if ( gcmd_line.display_info ) {
                printf("\n\nEnforced Hill-climbing failed !");
                printf("\nswitching to Best-first Search now.\n");
            }
            firstStep = do_best_first_search();
#if defined(PYTHON)
            if(firstStep && pythonMode)
                return get_special_plan(firstStep);
#endif
        }
        break;
    case 1:
    case 2:
        firstStep = do_best_first_search();
#if defined(PYTHON)
        if(firstStep && pythonMode)
            return get_special_plan(firstStep);
#endif
        break;
    case 3:
        firstStep = do_weighted_Astar();
#if defined(PYTHON)
        if(firstStep && pythonMode)
            return get_special_plan(firstStep);
#endif
        break;
    case 4:
        firstStep = do_Astar_epsilon();
#if defined(PYTHON)
        if(firstStep && pythonMode)
            return get_special_plan(firstStep);
#endif
        break;
    case 5:
        /* gcost_rplans controls whether or not we compute cost-minimal relaxed plans
         * gcost_minimizing is only used in h fn to decide whether or not we
         * need to count the weights of the operators in the relaxed plan.
         *
         * gcost_rplans may be false even for search options 3,4,5, namely if there are
         * numeric preconditions/goals which make this relaxed plan variant invalid.
         * hence we need to remember, when switching it off for EHC, whether or not
         * it was previously on.
         */
        prev_gcost_rplans = gcost_rplans;
        gcost_rplans = FALSE;
        gcost_minimizing = FALSE;
        found_plan = do_enforced_hill_climbing();
        if ( found_plan ) {
            if ( gcmd_line.display_info )
                print_plan();
#if defined(PYTHON)
            if(pythonMode)
                return get_regular_plan();
#endif
        } else {
            if ( gcmd_line.display_info ) {
                printf("\n\nEnforced Hill-climbing not successful.");
                printf("\nSwitching to A*epsilon now.");
            }
            gcost_rplans = prev_gcost_rplans;
            gcost_minimizing = TRUE;
            firstStep = do_Astar_epsilon();
#if defined(PYTHON)
            if(firstStep && pythonMode)
                return get_special_plan(firstStep);
#endif
        }
        break;
    default:
#if defined(PYTHON)
        if(pythonMode){
            PyErr_SetString(FFError, "Unknown search configuration! Valid values are from 0 to 5.");
            return NULL;
        }
#endif
        printf("\n\nUnknown search configuration: %d\n\n", gcmd_line.search_config );
        exit( 1 );
    }

    times( &end );
    TIME( gsearch_time );
#if defined(PYTHON)
    Py_INCREF(Py_None);
    return Py_None;
#endif
}






void output_planner_info( void )

{

    printf( "\n\ntime spent: %7.2f seconds instantiating %d easy, %d hard action templates",
            gtempl_time, gnum_easy_templates, gnum_hard_mixed_operators );
    printf( "\n            %7.2f seconds reachability analysis, yielding %d facts and %d actions",
            greach_time, gnum_pp_facts, gnum_actions );
    printf( "\n            %7.2f seconds creating final representation with %d relevant facts, %d relevant fluents",
            grelev_time, gnum_relevant_facts, gnum_relevant_fluents );
    printf( "\n            %7.2f seconds computing LNF",
            gLNF_time );
    printf( "\n            %7.2f seconds building connectivity graph",
            gconn_time );
    printf( "\n            %7.2f seconds searching, evaluating %d states, to a max depth of %d",
            gsearch_time, gevaluated_states, gmax_search_depth );
    printf( "\n            %7.2f seconds total time",
            gtempl_time + greach_time + grelev_time + gLNF_time + gconn_time + gsearch_time );

    printf("\n\n");
}



void ff_usage( void )

{

    printf("\nusage of ff:\n");

    printf("\nOPTIONS   DESCRIPTIONS\n\n");
    printf("-p <str>    Path for operator and fact file\n");
    printf("-o <str>    Operator file name\n");
    printf("-f <str>    Fact file name\n\n");

    printf("-r <int>    Random seed [used for random restarts; preset: 0]\n\n");

    printf("-s <int>    Search configuration [preset: s=5]; '+H': helpful actions pruning\n");
    printf("      0     Standard-FF: EHC+H then BFS (cost minimization: NO)\n");
    printf("      1     BFS (cost minimization: NO)\n");
    printf("      2     BFS+H (cost minimization: NO)\n");
    printf("      3     Weighted A* (cost minimization: YES)\n");
    printf("      4     A*epsilon (cost minimization: YES)\n");
    printf("      5     EHC+H then A*epsilon (cost minimization: YES)\n");
    printf("-w <num>    Set weight w for search configs 3,4,5 [preset: w=5]\n\n");

    printf("-C          Do NOT use cost-minimizing relaxed plans for options 3,4,5\n\n");

    printf("-b <float>  Fixed upper bound on solution cost (prune based on g+hmax) or depth (hill-climbing); active only with cost minimization\n\n");

    if ( 0 ) {
        printf("-i <num>    run-time information level( preset: 1 )\n");
        printf("      0     only times\n");
        printf("      1     problem name, planning process infos\n");
        printf("    101     parsed problem data\n");
        printf("    102     cleaned up ADL problem\n");
        printf("    103     collected string tables\n");
        printf("    104     encoded domain\n");
        printf("    105     predicates inertia info\n");
        printf("    106     splitted initial state\n");
        printf("    107     domain with Wff s normalized\n");
        printf("    108     domain with NOT conds translated\n");
        printf("    109     splitted domain\n");
        printf("    110     cleaned up easy domain\n");
        printf("    111     unaries encoded easy domain\n");
        printf("    112     effects multiplied easy domain\n");
        printf("    113     inertia removed easy domain\n");
        printf("    114     easy action templates\n");
        printf("    115     cleaned up hard domain representation\n");
        printf("    116     mixed hard domain representation\n");
        printf("    117     final hard domain representation\n");
        printf("    118     reachability analysis results\n");
        printf("    119     facts selected as relevant\n");
        printf("    120     final domain and problem representations\n");
        printf("    121     normalized expressions representation\n");
        printf("    122     LNF: translated subtractions representation\n");
        printf("    123     summarized effects LNF  representation\n");
        printf("    124     encoded LNF representation\n");
        printf("    125     connectivity graph\n");
        printf("    126     fixpoint result on each evaluated state\n");
        printf("    127     1P extracted on each evaluated state\n");
        printf("    128     H set collected for each evaluated state\n");

        printf("\n-d <num>    switch on debugging\n\n");
    }

}



Bool process_command_line( int argc, char *argv[] )

{

    char option;

    while ( --argc && ++argv ) {
        if ( *argv[0] != '-' || strlen(*argv) != 2 ) {
            return FALSE;
        }
        option = *++argv[0];
        switch ( option ) {
        /*     case 'E': */
        /*       gcmd_line.ehc = FALSE; */
        /*       break; */
        /*     case 'O': */
        /*       gcmd_line.optimize = TRUE; */
        /*       gcmd_line.ehc = FALSE; */
        /*       break;       */
        case 'C':
            gcmd_line.cost_rplans = FALSE;
            break;
        default:
            if ( --argc && ++argv ) {
                switch ( option ) {
                case 'p':
                    strncpy( gcmd_line.path, *argv, MAX_LENGTH );
                    break;
                case 'o':
                    strncpy( gcmd_line.ops_file_name, *argv, MAX_LENGTH );
                    break;
                case 'f':
                    strncpy( gcmd_line.fct_file_name, *argv, MAX_LENGTH );
                    break;
                case 'i':
                    sscanf( *argv, "%d", &gcmd_line.display_info );
                    break;
                case 'd':
                    sscanf( *argv, "%d", &gcmd_line.debug );
                    break;
                case 's':
                    sscanf( *argv, "%d", &gcmd_line.search_config );
                    break;
                case 'w':
                    sscanf( *argv, "%d", &gcmd_line.w );
                    break;
                case 'b':
                    sscanf( *argv, "%f", &gcmd_line.cost_bound );
                    break;
                default:
                    printf( "\nff: unknown option: %c entered\n\n", option );
                    return FALSE;
                }
            } else {
                return FALSE;
            }
        }
    }

    if ( 0 > gcmd_line.search_config || gcmd_line.search_config > 5 ) {
        printf("\n\nff: unknown search configuration %d.\n\n",
                gcmd_line.search_config);
        return FALSE;
    }

    if ( gcmd_line.search_config <= 2 ) {
        gcost_minimizing = FALSE;
        gcost_rplans = FALSE;
    } else {
        gcost_minimizing = TRUE;
        gcost_rplans = TRUE;
    }

    gw = gcmd_line.w;

    if ( !gcmd_line.cost_rplans ) {
        gcost_rplans = FALSE;
    }

    if ( gcmd_line.cost_bound != -1 && gcmd_line.cost_bound < 0 ) {
        printf("\n\nff: invalid cost bound %f; must be >= 0.\n\n",
                gcmd_line.cost_bound);
        return FALSE;
    }else if (gcmd_line.cost_bound > 0){
        glimit_search_depth = gcmd_line.cost_bound;
    }

    return TRUE;

}

#if defined(PYTHON)

static char* get_op_name( int index ){
    size_t size = 0;
    char* buffer = NULL;
    int i;
    Action *a = gop_conn[index].action;
    if ( !a->norm_operator && !a->pseudo_action ) {
        //printf("REACH-GOAL");
        size = snprintf(NULL, 0, "REACH-GOAL");
        buffer = calloc(size + 1, sizeof(char));
        if(!buffer)
            return NULL;
        sprintf(buffer, "REACH-GOAL");
    } else {
        size = snprintf(NULL, 0, "%s", a->name);
        for ( i = 0; i < a->num_name_vars; i++ )
            size += snprintf(NULL, 0, " %s", gconstants[a->name_inst_table[i]]);
        buffer = calloc(size + 1, sizeof(char));
        if(!buffer)
            return NULL;
        size = sprintf(buffer, "%s", a->name);
        for ( i = 0; i < a->num_name_vars; i++ ) {
            size += sprintf(buffer + size, " %s", gconstants[a->name_inst_table[i]]);
        }
    }
    return buffer;
}

static PyObject* get_regular_plan( void ){
    PyObject* result =  PyDict_New();
    if(!result){
        PyErr_SetString(FFError, "Could not create python dict");
        return NULL;
    }
    PyObject* actions =  PyDict_New();
    if(!actions){
        PyErr_SetString(FFError, "Could not create python dict");
        return NULL;
    }
    int i;
    float cost = 0;

    for ( i = 0; i < gnum_plan_ops; i++ ) {
        PyObject* key = PyInt_FromLong(i);
        if(!key){
          PyErr_SetString(FFError, "Could not create python int");
          return NULL;
        }
        char* name = get_op_name(gplan_ops[i]);
        if(!name){
            return PyErr_NoMemory();
        }
        PyObject* value = PyString_FromString(name);
        if(!value){
            PyErr_SetString(FFError, "Could not create python string");
            return NULL;
        }
        if(name){
            free(name);
        }
        if(PyDict_SetItem(actions, key, value) != 0){
            PyErr_SetString(FFError, "Could set python dict action item");
            return NULL;
        }
        Py_DecRef(key);
        Py_DecRef(value);
        if ( goptimization_established ) {
            cost += gop_conn[gplan_ops[i]].cost;
        }
    }
    if ( goptimization_established ) {
        PyObject* cost_value = PyFloat_FromDouble(cost);
        if(!cost_value){
            PyErr_SetString(FFError, "Could not parse python cost");
            return NULL;
        }
        if(PyDict_SetItemString(result, "cost", cost_value) != 0)
        {
            PyErr_SetString(FFError, "Could not set python cost");
            return NULL;
        }
        Py_DecRef(cost_value);
    }
    if(PyDict_SetItemString(result, "actions", actions) != 0){
        PyErr_SetString(FFError, "Could not set actions");
        return NULL;
    }
    Py_DecRef(actions);
    return result;
}

static PyObject* get_special_plan( BfsNode *last ){
    static int *ops = NULL;
    PyObject* result =  PyDict_New();
    if(!result){
        PyErr_SetString(FFError, "Could not create python dict");
        return NULL;
    }
    PyObject* actions =  PyDict_New();
    if(!actions){
        PyErr_SetString(FFError, "Could not create python dict");
        return NULL;
    }

    BfsNode *i;
    int j, num_ops;
    float cost;

    if ( get_special_plan_first_call ) {
        if(ops != NULL)
            free(ops);
        ops = ( int * ) calloc(MAX_PLAN_LENGTH, sizeof(int));
        if(!ops)
            return PyErr_NoMemory();
        get_special_plan_first_call = FALSE;
    }

    num_ops = 0;
    for ( i = last; i->op != -1; i = i->father ) {
        if ( num_ops == MAX_PLAN_LENGTH ){
            PyErr_SetString(FFError, "increase MAX_PLAN_LENGTH!");
            return NULL;
        }
        ops[num_ops++] = i->op;
    }

    gnum_plan_ops = 0;
    for ( j = num_ops - 1; j > -1; j-- ) {
        gplan_ops[gnum_plan_ops++] = ops[j];
    }

    cost = 0;
    for ( j = 0; j < gnum_plan_ops; j++ ) {
        PyObject* key = PyInt_FromLong(j);
        if(!key){
            PyErr_SetString(FFError, "Could not create python int");
            return NULL;
        }
        char* name = get_op_name(gplan_ops[j]);
        if(!name){
            return PyErr_NoMemory();
        }
        PyObject* value = PyString_FromString(name);
        if(!value){
            PyErr_SetString(FFError, "Could not create python string");
            return NULL;
        }
        if(name){
            free(name);
        }
        if(PyDict_SetItem(actions, key, value) != 0){
            PyErr_SetString(FFError, "Could set python dict action item");
            return NULL;
        }
        Py_DecRef(key);
        Py_DecRef(value);
        if ( goptimization_established ) {
            cost += gop_conn[gplan_ops[j]].cost;
        }
    }
    if ( goptimization_established ) {
        PyObject* cost_value = PyFloat_FromDouble(cost);
        if(!cost_value){
            PyErr_SetString(FFError, "Could not parse python cost");
            return NULL;
        }
        if(PyDict_SetItemString(result, "cost", cost_value) != 0){
            PyErr_SetString(FFError, "Could not set python cost");
            return NULL;
        }
        Py_DecRef(cost_value);
    }
    if(PyDict_SetItemString(result, "actions", actions) != 0){
        PyErr_SetString(FFError, "Could not set actions");
        return NULL;
    }
    Py_DecRef(actions);
    return result;
}

PyObject* ff_plan(PyObject* self, PyObject* args, PyObject* kw){

    cleanup();
    const char* domainPDDL = "";
    const char* problemPDDL = "";
    static char* keywords[] = {"domainPDDL", "problemPDDL", "searchMode", "weight", "upperCostBound", "costMinimization", "debug", NULL};
    int i;
    float cost;

    times ( &lstart );

    /* command line treatment
     */
    gcmd_line.display_info = 0;
    gcmd_line.debug = 0;

    /* search settings
     */
    gcmd_line.search_config = 5;
    gcmd_line.cost_rplans = TRUE;
    gcmd_line.w = 5;
    gcmd_line.cost_bound = -1;

    if (!PyArg_ParseTupleAndKeywords(args, kw, "ss|iifii", keywords, &domainPDDL, &problemPDDL, &gcmd_line.search_config, &gcmd_line.w, &gcmd_line.cost_bound, &gcmd_line.cost_rplans, &gcmd_line.debug))
    {
      PyErr_SetString(PyExc_ValueError, "Could not parse keywords");
      return NULL;
    }

    // printf("domain: %s\nproblem: %s\nsearch mode: %d\nweight: %d\ncost bound: %f\ncostMinimization: %d\ndebug: %d\n", domainPDDL, problemPDDL, gcmd_line.search_config, gcmd_line.w, gcmd_line.cost_bound, gcmd_line.cost_rplans, gcmd_line.debug);
    if (0 > gcmd_line.search_config || gcmd_line.search_config > 5) {
        PyErr_SetString(PyExc_ValueError, "unknown search configuration. Must be 0 to 5.");
        return NULL;
    }

    if ( gcmd_line.search_config <= 2 ) {
        gcost_minimizing = FALSE;
        gcost_rplans = FALSE;
    } else {
        gcost_minimizing = TRUE;
        gcost_rplans = TRUE;
    }

    gw = gcmd_line.w;

    if ( !gcmd_line.cost_rplans ) {
        gcost_rplans = FALSE;
    }

    if ( gcmd_line.cost_bound != -1 && gcmd_line.cost_bound < 0 ) {
        PyErr_SetString(PyExc_ValueError, "invalid cost bound. Must be >= 0");
        return NULL;
    }else if (gcmd_line.cost_bound > 0){
        glimit_search_depth = gcmd_line.cost_bound;
    }


    /* start parse & instantiation timing */
    times( &start );

    /* domain file (ops)
     */
    if ( gcmd_line.display_info >= 1 ) {
        printf("\nff: parsing domain");
    }

    /* it is important for the pddl language to define the domain before
     * reading the problem
     */
    if(parse_ops_from_memory(domainPDDL) != 0){
        return NULL; // error has already been set.
    }

    /* problem file (facts)
     */
    if ( gcmd_line.display_info >= 1 ) {
        printf(" ... done.\nff: parsing problem");
    }

    if(parse_fct_from_memory(problemPDDL) != 0){
        return NULL; // error has already been set.
    }
    if ( gcmd_line.display_info >= 1 ) {
        printf(" ... done.\n\n");
    }

    return doActualPlanning(TRUE);
}


static PyMethodDef FFMethods[] = {
        {"plan",  ff_plan, METH_VARARGS | METH_KEYWORDS, "Execute the planner."},
        {NULL, NULL, 0, NULL}        /* Sentinel */
};

PyMODINIT_FUNC initff(void){
    PyObject *m;

    m = Py_InitModule("ff", FFMethods);
    if (m == NULL)
        return;

    FFError = PyErr_NewException("ff.error", NULL, NULL);
    Py_INCREF(FFError);
    PyModule_AddObject(m, "error", FFError);
}

#ifdef DEBUG
int dynamicLoadCounter = 0;

__attribute__((constructor)) void dlentry(void) {
	printf("SO loaded. Counter: %d\n", ++dynamicLoadCounter);
}

__attribute__((destructor)) void dlexit(void) {
	printf("SO unloaded. Counter :%d\n", --dynamicLoadCounter);
}
#endif

#endif
