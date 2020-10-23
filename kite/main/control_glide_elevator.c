float control_glide_elevator(float P, float I, float D)
{
    float D_y = gyroy - avgGyroy;
    return -0.5 * D * D_y - 17 + 50 * elev_trim;
}