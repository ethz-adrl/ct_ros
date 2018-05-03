#include <ct/ros/vis/visualizer/colorMapping.h>

namespace ct{
namespace ros{

std_msgs::ColorRGBA getColor(float v, float vmin, float vmax)
{
   std_msgs::ColorRGBA c;

   // start with white
   c.a = 1.0;
   c.r = 1.0;
   c.g = 1.0;
   c.b = 1.0;

   if (v < vmin)
      v = vmin;
   if (v > vmax)
      v = vmax;
   float dv = vmax - vmin;

   if (v < (vmin + 0.25 * dv)) {
      c.r = 0;
      c.g = 4 * (v - vmin) / dv;
   } else if (v < (vmin + 0.5 * dv)) {
      c.r = 0;
      c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
   } else if (v < (vmin + 0.75 * dv)) {
      c.r = 4 * (v - vmin - 0.5 * dv) / dv;
      c.b = 0;
   } else {
      c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
      c.b = 0;
   }

   return c;
}

std_msgs::ColorRGBA getColor(int v, int vmin, int vmax)
{
   return getColor(static_cast<float> (v), static_cast<float> (vmin), static_cast<float> (vmax));
}

std_msgs::ColorRGBA getColor(bool v, bool min, bool max)
{
   std_msgs::ColorRGBA c;

  // start with white
  c.a = 1.0;
  c.r = static_cast<float>(v);
  c.g = static_cast<float>(v);
  c.b = static_cast<float>(v);

  return c;
}

} // namespace rbd
} // namespace ct

