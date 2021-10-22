#include "problem_instance.h"

std::ostream& operator<<(std::ostream& str, warthog::problem_instance& pi)
{
    pi.print(str);

    return str;
}
