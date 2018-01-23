/* scaling.h DD20151111 */
/**
 * @file scaling.h
 * @author Dusan Drevensek (DD)
 * @date 20150930
 * @brief File containing definitions for.
 *
 * Here typically goes a more extensive explanation of what the header
 * defines. Doxygens tags are words preceeded by either a backslash @\
 * or by an at symbol @@.
 * @see http://www.stack.nl/~dimitri/doxygen/docblocks.html
 * @see http://www.stack.nl/~dimitri/doxygen/commands.html
 */

#ifndef SCALING_H
#define SCALING_H

/* norm values for inverter */
#define NORM_VOLTAGE		48.	/* Voltage norm */
#define NORM_CURRENT		20.	/* Current norm */
#define NORM_TEMPERATURE    327.68	/* Temperature norm */
#define NORM_WE			6000.	/*7 Electrical angular speed norm */
#define NORM_WM			NORM_WE.	/* Mechanical angular speed norm */

#define NORM_IMPEDANCE  (NORM_VOLTAGE/NORM_CURRENT)
#define NORM_INDUCTANCE	(NORM_IMPEDANCE/NORM_WE)
#define NORM_POWER	    (NORM_VOLTAGE*NORM_CURRENT)
#define NORM_TORQUE		(NORM_POWER/NORM_WM)
#define NORM_FLUX			(NORM_VOLTAGE/NORM_WE)
#define NORM_ANGLE		    PI
#define NORM_TIME			(1./NORM_WE)


#define K_TIME_LONG			(NORM_TIME/(1L<<16))

/* scaling factors for DCAC */
#define K16_VOLTAGE		(NORM_VOLTAGE/(1L<<15))
#define K16_CURRENT		(double)(NORM_CURRENT/(double)(1L<<15))
#define K16_WE  		(NORM_WE/(1L<<15))
#define K16_WM  		(NORM_WM/(1L<<15))
#define K16_FLUX		(NORM_FLUX/(1L<<11))
#define K16_POWER		(NORM_POWER/(1L<<16))
#define K16_TORQUE		(NORM_TORQUE/(1L<<13))
#define K16_IMPEDANCE	(NORM_IMPEDANCE/(1L<<15))
#define K16_ANGLE		(NORM_ANGLE/(1L<<15))
#define K16_TIME		(NORM_TIME/(1L<<(15-2)))
#define K16_INDUCTANCE  (NORM_INDUCTANCE/(1L<<11))
#define K16_TEMPERATURE (TEMPERATURE_N/(1L<<15))



/* scaling of some specific system constants */

#define SAMPLING_TIME ((int16_t)(SAMPLING_TIME_FLOAT/K16_TIME))



#endif /* SCALING_H */
