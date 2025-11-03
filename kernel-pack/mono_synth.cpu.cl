/*
* mono_synth.cpu.cl
*
*  date    : 20.07.2017
*  author  : M. Khaled | Hybrid control systems @ Technical University of Munich, Germany
*  about   : an OpenCL kernel (optimized for CPUs) used to construct symbolic abstraction of
* 	         dynamical systems using [Growth Bound] and synthesis symbolic controllers using some
*			 fixed-point (FP) operations on the symbolic model constructed by over-approximating
*			 the reachabile sets (OARS).
* ***********************************************************************
*  The kernel manger will replace parameters enclosed by "@@" before compiling !
*/

// pfaces-Including a parameters and some funcs for the mono_synth
@pfaces-include:"mono_synth_utils.cl"

// pFaces-Including a KERNEL-Function: ABSTRACT	--- parallel algorrithm in (X,U)
@pfaces-include:"mono_synth_abstract.cl"

// pFaces-Including a KERNEL-Function: CHECK_SAFE_SET	--- parallel safe set checking
@pfaces-include:"mono_synth_safeset_check.cl"

