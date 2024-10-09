/*

En R2015a para Windows tuve que seleccionar 'Microsoft Windows SDK 7.1', tanto para C como para C++:
 
mex -setup:'C:\Program Files\MATLAB\MATLAB Production Server\R2015a\bin\win64\mexopts\winsdk-7.1_c.xml' C
mex -setup:'C:\Program Files\MATLAB\MATLAB Production Server\R2015a\bin\win64\mexopts\winsdk-7.1_cpp.xml' C++
 
ver https://la.mathworks.com/help/simulink/sfg/simstruct_introduction.html
*/
#include "../tst/debug.hpp"
#include <stdarg.h>

#define _DBG_TRACE
#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  N3_fcc

namespace {
unsigned time_stamp = 0;
}

#ifdef MEX_FILE_CHECK
namespace {

	typedef int int_T;
	typedef double real_T;
	typedef double **InputRealPtrsType;

	struct SimStruct {};
	struct mxArray {};
	typedef int mwSize;

	int ssSetNumSFcnParams             (SimStruct*, int)      { return 1; }
	int ssGetNumSFcnParams             (SimStruct*)           { return 1; }
	int ssGetSFcnParamsCount           (SimStruct*)           { return 1; }
	int ssSetNumInputPorts             (SimStruct*, int)      { return 1; }
	int ssSetInputPortDataType         (SimStruct*, int, int) { return 1; }
	int ssSetInputPortVectorDimension  (SimStruct*, int, int) { return 1; }
	int ssSetInputPortDirectFeedThrough(SimStruct*, int, int) { return 1; }
	int ssSetNumOutputPorts            (SimStruct*, int)      { return 1; }
	int ssSetOutputPortVectorDimension (SimStruct*, int, int) { return 1; }
	int ssSetNumRWork                  (SimStruct*, int)      { return 1; }
	int ssSetNumIWork                  (SimStruct*, int)      { return 1; }
	int ssSetNumPWork                  (SimStruct*, int)      { return 1; }
	int ssSetNumModes                  (SimStruct*, int)      { return 1; }
	int ssSetNumNonsampledZCs          (SimStruct*, int)      { return 1; }
	int SetSimStateCompliance          (SimStruct*, int)      { return 1; }
	int ssSetNumContStates             (SimStruct*, int)      { return 1; }
	int ssSetNumDiscStates             (SimStruct*, int)      { return 1; }
	int ssSetNumSampleTimes            (SimStruct*, int)      { return 1; }
	int ssSetOptions                   (SimStruct*, int)      { return 1; }
	const char* ssGetErrorStatus       (SimStruct*)           { return nullptr; }
	void        ssSetErrorStatus       (SimStruct*, const char*) { }

    int ssSetSampleTime(SimStruct*, int, double)              { return 1; }
    int ssSetOffsetTime(SimStruct*, int, double)              { return 1; }

	#define DYNAMICALLY_TYPED                1
	#define INHERITED_SAMPLE_TIME           -1
	#define SS_OPTION_WORKS_WITH_CODE_REUSE  1
	#define SS_OPTION_EXCEPTION_FREE_CODE    2
	#define SS_OPTION_CALL_TERMINATE_ON_EXIT 4
	#define UNUSED_ARG(x)

	mxArray* ssGetSFcnParam         (SimStruct*, int)         { return 0; }
    int      mxGetNumberOfDimensions(const mxArray*)          { return 1; }
    int*     mxGetDimensions        (const mxArray*)          { return 0; }

	int      ssGetNumPWork(SimStruct*)                        { return 1; }
	void**   ssGetPWork   (SimStruct*)                        { return 0; }
	const real_T* mxGetPr (const mxArray*)                    { return 0; }

    int      ssGetOutputPortWidth        (SimStruct*, int)    { return 1; }
    real_T*  ssGetOutputPortRealSignal   (SimStruct*, int)    { return 0; }
    int      ssGetInputPortWidth         (SimStruct*, int)    { return 1; }
    real_T** ssGetInputPortRealSignalPtrs(SimStruct*, int)    { return 0; }

    void ssPrintf(const char *fmt, ...) {}
}
#else
	#include "simstruc.h"

    namespace gnc {

    void trace(const char* msg)
    {
    	ssPrintf("%04d: [MEX] %s\n", time_stamp, msg);
    }

    void debug_print(int dbg_lvl, const char *fmt, ...)
    {
    	char s_buffer[256]; sprintf(s_buffer, "%04d: [FCC] ", time_stamp);
    	int hdr = strlen(s_buffer);
    	va_list  args;
    	va_start(args,fmt);
    	int cnt  = vsnprintf(s_buffer+hdr, sizeof(s_buffer)-hdr, fmt, args);
    	va_end  (args);
    	if(cnt < sizeof(s_buffer)-hdr)
    	{
    		cnt += hdr;
    		s_buffer[cnt] = '\n';
    		s_buffer[cnt+1] = 0;
    		ssPrintf(s_buffer);
    	}
    }
    void debug_print(int dbg_lvl, char* buf, size_t len, const char *fmt, ...)
    {
    	sprintf(buf, "%04d: [FCC] ", time_stamp);
    	int hdr = strlen(buf);
    	va_list  args;
    	va_start(args,fmt);
    	int cnt = vsnprintf(buf+hdr, len-hdr, fmt, args);
    	va_end  (args);
    	if(cnt < len-hdr)
    	{
    		cnt += hdr;
    		buf[cnt] = '\n';
    		buf[cnt+1] = 0;
    		ssPrintf(buf);
    	}
    }

    }
#endif
#ifdef _DBG_TRACE
	#define _TRACE(x) ssPrintf("%04d: [MEX] " x "\n", time_stamp)
#else
	#define _TRACE(x)
#endif
#define _ERROR(S, x) ssPrintf("%04d: [ERROR] %s\n", time_stamp, x); ssSetErrorStatus(S, x);

namespace {

int get_length(const mxArray* p_param)
{
	mwSize         n_dim = mxGetNumberOfDimensions(p_param);
	const mwSize * p_dim = mxGetDimensions(p_param);
	if(n_dim != 2)
		return 0;
	return std::max(*p_dim,*(p_dim+1));
}
int get_cols(const mxArray* p_param)
{
	mwSize         n_dim = mxGetNumberOfDimensions(p_param);
	const mwSize * p_dim = mxGetDimensions(p_param);
	if(n_dim != 2)
		return 0;
	return *(p_dim+1);
}

}

static void mdlInitializeSizes      (SimStruct *S);
static void mdlInitializeSampleTimes(SimStruct *S);
#define     MDL_START
static void mdlStart                (SimStruct *S);

#define     MDL_CHECK_PARAMETERS
#define     MDL_PROCESS_PARAMETERS 
static void mdlCheckParameters      (SimStruct *S);
static void mdlProcessParameters    (SimStruct *S);

//#define     MDL_INITIALIZE_CONDITIONS
//static void mdlInitializeConditions (SimStruct *S);

static void mdlOutputs              (SimStruct *S, int_T tid);

static void mdlTerminate            (SimStruct *S);

// #define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
// !mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

//    The sizes information is used by Simulink to determine the S-function 
//    block's characteristics (number of inputs, outputs, states, etc.).  

static void mdlInitializeSizes(SimStruct *S)
{
#ifndef MEX_FILE_CHECK
	ssPrintf("[FCC] build " __DATE__ " " __TIME__);
#endif
	_TRACE("mdlInitializeSizes");
	//---------------------------------------------------------------
    // Se esperan 4 parámetros:
	//---------------------------------------------------------------
    ssSetNumSFcnParams(S, 4);
	// Los parámetros no se pueden modificar durante la simulación
    // 	ssSetSFcnParamTunable(S, 0, 0); // params
    // 	ssSetSFcnParamTunable(S, 1, 0); // gains
    // 	ssSetSFcnParamTunable(S, 2, 0); // wire
    // 	ssSetSFcnParamTunable(S, 3, 0); // initial state
    if(ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
        // Parameter mismatch will be reported by Simulink
        return;
    
	//---------------------------------------------------------------
	// Entradas: 
	//---------------------------------------------------------------
    if(!ssSetNumInputPorts(S, 1))
		return;
    ssSetInputPortDataType         (S, 0, DYNAMICALLY_TYPED);
    ssSetInputPortVectorDimension  (S, 0, gnc::fcc_dbg_t::sns_len);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    
	//---------------------------------------------------------------
	// Salidas:
	//---------------------------------------------------------------
    if(!ssSetNumOutputPorts(S, 3))
		return;
    ssSetOutputPortVectorDimension(S, 0, gnc::fcc_dbg_t::aos_len);
    ssSetOutputPortVectorDimension(S, 1, gnc::fcc_dbg_t::dos_len);
    ssSetOutputPortVectorDimension(S, 2, gnc::fcc_dbg_t::out_len);
    
	//---------------------------------------------------------------
	// Auxiliares:
    // Reserve place for C++ object
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 1); 
    ssSetNumModes(S, 0);
	
	//---------------------------------------------------------------
	// Estados:
    ssSetNumNonsampledZCs  (S, 0);
    //SetSimStateCompliance(S, USE_CUSTOM_SIM_STATE);
    ssSetNumContStates     (S, 0);
    ssSetNumDiscStates     (S, 0);  // track, last error vect
    
    ssSetNumSampleTimes    (S, 1);

	// If you do not call mexErrMsgTxt or any other routines that cause exceptions,
	// then you should use SS_OPTION_EXCEPTION_FREE_CODE S-function option.  
	ssSetOptions(S, SS_OPTION_WORKS_WITH_CODE_REUSE 
                  | SS_OPTION_EXCEPTION_FREE_CODE 
                  | SS_OPTION_CALL_TERMINATE_ON_EXIT );
    
	//---------------------------------------------------------------
    // You should add a call to mdlCheckParameters from mdlInitalizeSizes
    // to check the parameters. After setting the number of parameters
    // you expect in your S-function via ssSetNumSFcnParams(S,n), you should:
    if(ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) 
    {
       mdlCheckParameters(S);
       if(ssGetErrorStatus(S) != NULL) 
           return;
    } 
    else 
    	return; // Simulink will report a parameter mismatch error   
}

//   This function is used to specify the sample time(s) for your			   
//   S-function. You must register the same number of sample times as		   
//   specified in ssSetNumSampleTimes.										   
static void mdlInitializeSampleTimes(SimStruct *S)
{
	_TRACE("InitializeSampleTimes");

	//ssWarning(S, "mdlInitializeSampleTimes");    
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    //ssSetModelReferenceSampleTimeDefaultInheritance(S); 
}

//   This function is called once at start of model execution. If you		 
//   have states that should be initialized once, this is the place			 
//   to do it.	
static void mdlStart(SimStruct *S)
{
	_TRACE("mdlStart");
    // Store new C++ object in the pointers vector
    ssGetPWork(S)[0] = new gnc::fcc_dbg_t();
	mdlProcessParameters(S);
}

//     mdlCheckParameters verifies new parameter settings whenever parameter
//     change or are re-evaluated during a simulation. When a simulation is
//     running, changes to S-function parameters can occur at any time during
//     the simulation loop.
// 
//     This method can be called at any point after mdlInitializeSizes.
// 
//     When a Simulation is running, changes to S-function parameters can
//     occur either at the start of a simulation step, or during a
//     simulation step. When changes to S-function parameters occur during
//     a simulation step, this method is called twice, for the same
//     parameter changes. The first call during the simulation step is
//     used to verify that the parameters are correct. After verifying the
//     new parameters, the simulation continues using the original
//     parameter values until the next simulation step at which time the
//     new parameter values will be used. Redundant calls are needed to
//     maintain simulation consistency.  Note that you cannot access the
//     work, state, input, output, etc. vectors in this method. This
//     method should only be used to validate the parameters. Processing
//     of the parameters should be done in mdlProcessParameters.
// 
//     See matlabroot/simulink/src/sfun_errhdl.c for an example. 

//=============================================================================
// Se esperan tres parámetros:
//
// 	params: [ ]
// 	gains:
// 	wire:
//
//=============================================================================
#ifdef MDL_CHECK_PARAMETERS
static void mdlCheckParameters(SimStruct *S)
{   
	_TRACE("mdlCheckParameters");

	if(get_length(ssGetSFcnParam(S, 0)) != gnc::fcc_dbg_t::prm_len)
    {
        //_TRACE("n_dim:%s, dim(1):%s, dim(2):%s", n_dim, *p_dim, *(p_dim+1));
        ssSetErrorStatus(S, "Parameter 1 'params' data must be a vector with 17 elements "
							"\nstage, R_orbit"
							"\ntimes   { sample, s1_burn, s2_burn, meco_duration }"
							"\ndelays  { separation, burn }"
							"\nheights { relief, void, separation, steer }"
							"\npguid   { V_exit, pre-cycle, tgo_min, v_eps, f_min }");
		return;
	}
	if(get_cols(ssGetSFcnParam(S, 1)) != 4)
    {
        ssSetErrorStatus(S, "Parameter 2 'gains' data must be an array with 4 columns");
        return;
	}
	if(get_cols(ssGetSFcnParam(S, 2)) != 2)
    {
        ssSetErrorStatus(S, "Parameter 3 'wire' data must be an array with 2 columns [pitch , heading]");
        return;
	}
	if(get_length(ssGetSFcnParam(S, 3)) != 3 && get_length(ssGetSFcnParam(S, 3)) != 0)
    {
        ssSetErrorStatus(S, "Parameter 4 'initial state' must have 3 elements [state , t_launch, t_separation]");
        return;
    }
	_TRACE("mdlCheckParameters OK");    
}
#endif

//     This method will be called after mdlCheckParameters, whenever
//     parameters change or get re-evaluated. The purpose of this method is
//     to process the newly changed parameters. For example "caching" the
//     parameter changes in the work vectors. Note this method is not
//     called when it is used with the Real-Time Workshop. Therefore,
//     if you use this method in an S-function which is being used with the
//     Real-Time Workshop, you must write your S-function such that it doesn't
//     rely on this method. This can be done by inlining your S-function
//     via the Target Language Compiler.

//=============================================================================
// Parámetros                                                                  
//
//
//
//
//=============================================================================
#ifdef MDL_PROCESS_PARAMETERS 
static void mdlProcessParameters(SimStruct *S)
{
	_TRACE("mdlProcessParameters");
	try
	{
		gnc::fcc_dbg_t* p_dbg = static_cast<gnc::fcc_dbg_t*>(ssGetPWork(S)[0]);
		if(!p_dbg)
		{
			_TRACE("PWork is null");
			return;
		}

		const mxArray* p_param;
		const real_T * p_data;

		// ------------------------------------ Parámetros
		p_param  = ssGetSFcnParam(S, 0);
		p_data   = mxGetPr(p_param);
		if(p_data)
		{
			p_dbg->fill_params(p_data, get_length(p_param));
	//		ssPrintf(o_fcc.man.str_config().c_str());
		}
		else
			ssSetErrorStatus(S, "Invalid parameter 1 data");

		// ------------------------------------ Ganancias
		p_param = ssGetSFcnParam(S, 1);
		p_data  = mxGetPr(p_param);
		if(p_data)
		{
			p_dbg->fill_gains(p_data, get_length(p_param));
		}

		// ------------------------------------ Wire
		p_param = ssGetSFcnParam(S, 2);
		p_data  = mxGetPr(p_param);
		if(p_data)
		{
			p_dbg->fill_wire(p_data, get_length(p_param));
		}
		else
			ssSetErrorStatus(S, "Invalid parameter 2 data");
        _TRACE("mdlProcessParameters Done");

		// ------------------------------------ Initial state
		p_param = ssGetSFcnParam(S, 3);
		if(p_param)
		{
			p_data  = mxGetPr(p_param);
			if(p_data && get_length(p_param) >= 3)
				p_dbg->initial_state(*p_data, *(p_data+1), *(p_data+2));
		}
		_TRACE("mdlProcessParameters Done");
	}
	catch(const std::exception& e)
	{
        _ERROR(S, e.what())
	}
}
#endif

////    Initialize both discrete states to one.
//static void mdlInitializeConditions(SimStruct *S)
//{
//    // real_T *p = ssGetRealDiscStates(S);
//	// for(real_T *e = p + 4; p<e; ++p)
//        // *p = 0;
//}

inline dgnc::geom::scalar pop_scalar(InputRealPtrsType& i)
{
	dgnc::geom::scalar x = **(i); ++i;
	return x;
}
inline dgnc::geom::vector pop_vector(InputRealPtrsType& i)
{
	double x = **(i); ++i;
	double y = **(i); ++i;
	double z = **(i); ++i;
	return dgnc::geom::vector(x, y, z);
}

//=============================================================================
static void mdlOutputs(SimStruct *S, int_T tid)
{
    try
    {
    	using namespace gnc;
    	++time_stamp;
        UNUSED_ARG(tid);
        fcc_dbg_t*  p_dbg = static_cast<fcc_dbg_t*>(ssGetPWork(S)[0]);
        fcc_t&      o_fcc = p_dbg->fcc;
        ins_data_t& o_ins = p_dbg->ins;

        InputRealPtrsType i_ins = ssGetInputPortRealSignalPtrs(S,0);
        int_T           ins_len = ssGetInputPortWidth (S,0);

        if(ins_len >= fcc_dbg_t::sns_len)
        {
        	//using namespace dgnc::geom;
			o_ins.elapsed = pop_scalar(i_ins);
			o_ins.pos     = pop_vector(i_ins);
			o_ins.vel     = pop_vector(i_ins);
			scalar re     = pop_scalar(i_ins);
			vector im     = pop_vector(i_ins);
			o_ins.att     = quaternion(re, im);
			o_ins.wbi     = pop_vector(i_ins);
			o_ins.sfc     = pop_vector(i_ins);
			o_ins.lla.lat = pop_scalar(i_ins);
			o_ins.lla.lng = pop_scalar(i_ins);
			o_ins.lla.alt = pop_scalar(i_ins);

            p_dbg->on_time_step();

            real_T*
            o_ptr = ssGetOutputPortRealSignal(S, 0);
            if(ssGetOutputPortWidth(S,0) >= fcc_dbg_t::aos_len)
            {
                *(  o_ptr) = o_fcc.AOs().dy;
                *(++o_ptr) = o_fcc.AOs().dz;
                *(++o_ptr) = o_fcc.AOs().da;
                *(++o_ptr) = o_fcc.AOs().rc;
            }
            o_ptr = ssGetOutputPortRealSignal(S, 1);
            if(ssGetOutputPortWidth(S,1) >= fcc_dbg_t::dos_len)
            {
                *(  o_ptr) = o_fcc.DOs().eng;
                *(++o_ptr) = o_fcc.DOs().rel;
                *(++o_ptr) = o_fcc.DOs().sep;
            }
            o_ptr = ssGetOutputPortRealSignal(S, 2);
            if(ssGetOutputPortWidth(S,2) >= fcc_dbg_t::out_len)
                p_dbg->write_tlmy(o_ptr);
        }
        else
        {
            _TRACE("mdlOutputs::invalid inputs");
        }
    }
    catch(...)
    {
        _TRACE("mdlOutputs::catch");
    }
}

//   In this function, you should perform any actions that are necessary	   
//   at the termination of a simulation.  For example, if memory was		   
//   allocated in mdlStart, this is the place to free it.					   
static void mdlTerminate(SimStruct *S)
{
	_TRACE("mdlTerminate");

    if(ssGetNumPWork(S) == 1)
    {
        // Retrieve and destroy C++ object 
        void **pw = ssGetPWork(S);
        if(!pw)
            return;
        gnc::fcc_dbg_t *p = static_cast<gnc::fcc_dbg_t*>(*pw);
        ssGetPWork(S)[0] = 0;
        if(p != NULL)
            delete p;
    }
}

#ifndef MEX_FILE_CHECK
// Required S-function trailer
#ifdef  MATLAB_MEX_FILE    // Is this file being compiled as a MEX-file? 
#include "simulink.c"      // MEX-file interface mechanism 
#else
#include "cg_sfun.h"       // Code generation registration function 
#endif
#endif
