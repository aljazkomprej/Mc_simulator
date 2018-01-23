/** \file structs.h
 * \brief Header file defines some structures frequently used in control software.
 *
 * $URL: svn://192.168.1.56/solaris/banches/solaris3_with_flyback/piccolo/include/structs.h $
 * $Rev: 218 $
 * \author Dušan Drevenšek, TECES (Initial version 2012)
 * $Author: drevensekd $
 * $Date: 2014-05-07 14:17:17 +0200 (sre, 07 maj 2014) $
 *
 * \details
 * In order to simplify and unify data representation in software, some physical quantities are represented as
 * vectors. Same quantities could also be alternatively represented as complex numbers. To avoid double
 * representation, vectors are always presented in a form of complex numbers in software.
 *
 * Sometimes some quantities are represented as 16-bit and 32-bit numbers at the same time. This is a consequence of striving for
 * the better accuracy of calculation. For that reason there are some structures defined, that enable access to
 * the variables with both precisions.
 */

#ifndef STRUCTS_H_INCLUDED
#define STRUCTS_H_INCLUDED

#include <stdint.h>


/** \brief Structure represents 16-bit complex number (or vector). */
typedef struct
{
  /** \brief Real component of complex number or the first Cartesian component of a vector. */
  int16_t re;
  
  /** \brief Imaginary component of complex number or the second Cartesian component of a vector. */
  int16_t im;
  
  /** \brief Absolute value of a complex number or length of the vector. */
  uint16_t abs;
  
  /** \brief Angle of complex value. */
  int16_t angle;
} complex16_t;



/** \brief Structure represents 32-bit complex number (or vector). */
typedef struct
{
  /** \brief Real component of complex number or the first Cartesian component of a vector. */
  int32_t re;

  /** \brief Imaginary component of complex number or the second Cartesian component of a vector. */
  int32_t im;
} complex32_t;



/** \brief Structure represents angle and corresponding values of trigonometric functions sinus and cosinus. */
typedef struct
{
  /**  \brief Contains angle. */
  uint16_t angle;
  
  /** \brief Value of the sinus function. */
  int16_t sin;
  
  /** \brief Value of the cosinus function. */
  int16_t cos;
} angle_t;

#endif /* STRUCTS_H_INCLUDED */
