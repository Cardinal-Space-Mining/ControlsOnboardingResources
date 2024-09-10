#ifndef PS4_MAP_09_08_24
#define PS4_MAP_09_08_24

namespace Ps4Mapping
{
namespace Buttons
{
constexpr int CROSS = 0;
constexpr int CIRCLE = 1;
constexpr int TRIANGLE = 2;
constexpr int SQUARE = 3;
constexpr int LB = 4;
constexpr int RB = 5;
constexpr int L_TRIGGER = 6;
constexpr int R_TRIGGER = 7;
constexpr int SHARE = 8;
constexpr int OPTIONS = 9;
constexpr int PS = 10;
constexpr int L_STICK = 11;
constexpr int R_STICK = 12;

constexpr int NUM_BUTTONS = 13;

} // namespace Buttons

namespace Axes
{
constexpr int LEFTX = 0;
constexpr int LEFTY = 1;
constexpr int L_TRIGGER = 2;

constexpr int RIGHTX = 3;
constexpr int RIGHTY = 4;
constexpr int R_TRIGGER = 5;

constexpr int DPAD_R_L = 6;
constexpr int DPAD_U_D = 7;

namespace DPAD_K
{

constexpr double DPAD_DOWN = -1.0;
constexpr double DPAD_UP = 1.0;

constexpr double DPAD_RIGHT = -1.0;
constexpr double DPAD_LEFT = 1.0;
} // namespace DPAD_K

constexpr int NUM_AXES = 8;

} // namespace Axes
} // namespace LogitechMapping

#endif