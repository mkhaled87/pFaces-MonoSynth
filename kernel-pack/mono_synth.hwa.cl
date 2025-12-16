/*
* mono_synth.hwa.cl
*
*  date    : 20.07.2017
*  author  : M. Khaled | Hybrid control systems @ Technical University of Munich, Germany
*  about   : an OpenCL kernel (optimized for HWAs) used to construct symbolic abstraction of
* 	         dynamical systems using [Growth Bound] and synthesis symbolic controllers using some
*			 fixed-point (FP) operations on the symbolic model constructed by over-approximating
*			 the reachabile sets (OARS).
* ***********************************************************************
*  The kernel manger will replace parameters enclosed by "@@" before compiling !
*/

// pFaces-Including a KERNEL-Function: precompute_transitions	--- parallel algorithm in (X)
@pfaces-include:"precompute_transitions.cl"
