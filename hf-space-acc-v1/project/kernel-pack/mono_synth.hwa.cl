/*
* mono_synth.hwa.cl
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

// pFaces-Including KERNEL-Functions: bitmap_gfp_iterate + bitmap_gfp_advance + bitmap_gfp_prefix
@pfaces-include:"bitmap_gfp_iterate.cl"
@pfaces-include:"bitmap_gfp_advance.cl"
@pfaces-include:"bitmap_gfp_prefix.cl"
