/*
* mono_synth.cpu.cl
*
*  date    : 20.01.2026
*  about   : Soon.
* ***********************************************************************
*  The kernel manger will replace parameters enclosed by "@@" before compiling !
*/

// pFaces-Including a KERNEL-Function: precompute_transitions	--- parallel algorithm in (X)
@pfaces-include:"precompute_transitions.cl"

// pFaces-Including a KERNEL-Function: check_basis_safety
@pfaces-include:"check_basis_safety.cl"

// pFaces-Including a KERNEL-Function: build_bitmap
@pfaces-include:"build_bitmap.cl"
