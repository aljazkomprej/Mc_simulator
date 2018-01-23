/* park.h  DD20151111 */

#ifndef PARK_H
#define PARH_H

#ifdef __cplusplus
extern "C" {
#endif

void transf_dq_ab_fix16(const complex16_t *in, complex16_t *out, const angle_t *angle);
void transf_ab_dq_fix16(const complex16_t *in, complex16_t *out, const angle_t *angle);

#ifdef __cplusplus
}
#endif


#endif /* PARK_H */
