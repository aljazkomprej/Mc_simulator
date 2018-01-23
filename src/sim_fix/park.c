/* park.c  DD20151111 */
/** \file park.c
 * \brief Module contains the Park and Inverse Park transformation
 * \ingroup MCONTROL
 *
 * $URL: $
 * $Rev:  $
 * \author Dusan Drevensek, TECES (Initial version 20151111)
 * $Author: drevensekd $
 * $Date: $
*/


#include <stdint.h>
#include "structs.h"
#include "park.h"
#include "h430_mul_mac.h"


/** \brief Function transforms vector from (d,q) space to (a,b) space.
\author Dušan Drevenšek
\param *in  Pointer to input vector in (d,q) space.
\param *out Pointer to output vector in (a,b) space
\param *angle Pointer to ANGLE structure that holds angle with
sin & cos values (they need to be calculated before calling this function)

For transformation folowing formulae is used:
 - a=d*cos-q*sin
 - b=d*sin+q*cos

For all vector operations COMPLEX_INT structure is used to hold variables.
*/
void transf_dq_ab_fix16(const complex16_t *in, complex16_t *out, const angle_t *angle)
{
  out->re=MulsMacsAsr15_Sat( in->re,angle->cos,-in->im,angle->sin);
  out->im=MulsMacsAsr15_Sat( in->re,angle->sin, in->im,angle->cos);
}


/** \brief Function transforms vector from (a,b) space to (d,q) space.
\author Dušan Drevenšek
\param *in  Pointer to input vector in (a,b) space.
\param *out Pointer to output vector in (d,q) space
\param *angle Pointer to ANGLE structure that holds angle with
sin & cos values (they need to be calculated before calling this function)

For transformation folowing formulae is used:
 - d= a*cos+b*sin
 - q=-a*sin+b*cos

For all vector operations COMPLEX_INT structure is used to hold variables.
*/
void transf_ab_dq_fix16(const complex16_t *in, complex16_t *out, const angle_t *angle)
{
    out->re=MulsMacsAsr15_Sat( in->re,angle->cos, in->im,angle->sin);
    out->im=MulsMacsAsr15_Sat(-in->re,angle->sin, in->im,angle->cos);
}


/* DD   xxxxx  functions not tested yet  */
