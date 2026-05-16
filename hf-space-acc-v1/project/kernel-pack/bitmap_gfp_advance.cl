/*
 * bitmap_gfp_advance.cl
 *
 * Device-side buffer advance for bitmap GFP. Copies S_{k+1} to S_k and
 * clears the output buffer without host transfer.
 */

__kernel void bitmap_gfp_advance(
    __global uint* bitmap_in,
    __global uint* bitmap_out,
    __global int* changed_flag
) {
    const size_t word = get_global_id(0);
    if (word >= (size_t)(@@BITMAP_WORD_COUNT@@)) return;

    const uint next = bitmap_out[word];
    if (bitmap_in[word] != next) {
        atomic_or((volatile __global int*)changed_flag, 1);
    }
    bitmap_in[word] = next;
    bitmap_out[word] = 0u;
}
