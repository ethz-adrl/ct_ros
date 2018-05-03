/*
 * colorMapping.h
 *
 *  Created on: 14.10.2013
 *      Author: neunertm
 */

#ifndef COLORMAPPING_H_
#define COLORMAPPING_H_

#include <std_msgs/ColorRGBA.h>

namespace ct {
namespace ros {

/*
   Return a RGB colour value given a scalar v in the range [vmin,vmax]
   In this case each colour coctonent ranges from 0 (no contribution) to
   1 (fully saturated), modifications for other ranges is trivial.
   The colour is clipped at the end of the scales if v is outside
   the range [vmin,vmax]
*/

std_msgs::ColorRGBA getColor(float v, float vmin, float vmax);

std_msgs::ColorRGBA getColor(int v, int vmin, int vmax);

std_msgs::ColorRGBA getColor(bool v, bool min, bool max);

} // namespace ros
} // namespace ct



#endif /* COLORMAPPING_H_ */
